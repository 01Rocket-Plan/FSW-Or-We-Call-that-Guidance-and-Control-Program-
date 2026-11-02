// GnssSimDevice：TEL0157 的 NMEA 位元組流模擬器（給 GnssDriverSim 與 host 測試）
//
// 為何存在：Core 只認 IGnssUart；把「接收機」做成可注入的位元組源，才能在不開 F´ 的
//   主機測試裡驗證失鎖表、分頻、emit／seq、C6（飛行中失鎖不中止）。
// 安裝：DS／US 皆可，與真驅動相同。
// 原則：
//   - 軌跡「真值」只在本類別；Core 只能 readBytes 看到 NMEA。
//   - 量測 = 真值 + 雜訊 + 延遲 + 偶發掉 fix；不 sleep、不讀系統時鐘，時間由 advance(dt_s)。
//   - baud 信用額：每 advance 依 baud×dt/10 釋放位元組，模擬 8N1 線速（SLOW 2 s 堆積→2048 緩衝）。
// Gate C6：NO_FIX／NO_DATA／STUCK 等注入驗證降級與恢復，Core 不得進 FAULT。
#ifndef GNSS_GNSSSIMDEVICE_HPP
#define GNSS_GNSSSIMDEVICE_HPP
#include "Components/Gnss/GnssDriver/GnssIfaces.hpp"
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace Gnss {

// Sim 專用故障（與核心的 GnssFault 分開）
enum class GnssSimFault : uint8_t {
  NONE = 0,
  NO_FIX = 1,        // GGA 品質 0、GSA mode 1、RMC 狀態 V
  FIX_2D_ONLY = 2,   // GSA mode 2（只有 2D fix）
  BAD_CHECKSUM = 3,  // GGA 的校驗和故意算錯
  NO_DATA = 4,       // 完全不送位元組
  STUCK = 5,         // 一直重送注入當下那一組句子
  TRUNCATED = 6      // GGA 句子在中途被截斷（沒有 *hh\r\n）
};

class GnssSimDevice : public IGnssUart {
 public:
  GnssSimDevice();

  // 每個 tick 在核心 tick 之前呼叫；dt_s = 本拍時長（rg100Hz 時約 0.01；epoch 仍 1 Hz）
  void advance(double dt_s);
  int32_t readBytes(uint8_t* buf, uint32_t max) override;

  // 軌跡：起點（WGS84，弧度／公尺 MSL）＋ 固定 ENU 速度（m/s）
  void setTrajectory(double lat0_rad, double lon0_rad, double alt0_m, const float velEnu_mps[3]);
  // 雜訊標準差：水平位置、垂直位置（m）、水平速度（m/s）
  void setNoise(float posH_sigma_m, float posV_sigma_m, float vel_sigma_mps);
  // 輸出延遲：量測時刻到句子開始送出的延遲（s）
  void setLatency(double latency_s) { m_latency_s = latency_s; }
  // 每個 epoch 偶發掉 fix 的機率（0~1）
  void setDropoutProb(float p) { m_dropoutProb = p; }
  void setFault(GnssSimFault f);
  void setBaud(uint32_t baud) { m_baud = baud; }  // 預設 9600（8N1：每位元組 10 bit）
  // HDOP 偏置：加到漫步後的 HDOP（測試超門檻用）；最終仍夾在 0.1~99
  void setHdopBias(float bias) { m_hdopBias = bias; }

  // ---- 僅供 host 測試檢查用（不給核心）----
  double simTime_s() const { return m_t_s; }
  void truthAt(double t_s, double& lat_rad, double& lon_rad, double& alt_m) const;
  uint32_t epochsGenerated() const { return m_epochs; }

  static uint8_t nmeaChecksum(const std::string& body);  // body 不含 $ 與 *

 private:
  struct Pending { double release_s; std::string bytes; };
  void makeEpoch(double tEpoch_s);
  double gauss();
  uint32_t rnd();

  double m_t_s = 0.0;
  double m_nextEpoch_s = 1.0;  // 1 Hz 輸出
  double m_lat0 = 0.0, m_lon0 = 0.0, m_alt0 = 0.0;
  float m_vel[3] = {0.f, 0.f, 0.f};
  float m_sigPosH = 2.5f, m_sigPosV = 4.0f, m_sigVel = 0.1f;
  double m_latency_s = 0.15;
  float m_dropoutProb = 0.0f;
  GnssSimFault m_fault = GnssSimFault::NONE;
  uint32_t m_baud = 9600;
  double m_byteCredit = 0.0;
  float m_hdop = 1.0f;     // 隨機漫步 0.6~4.0
  float m_hdopBias = 0.0f;
  int m_sats = 10;         // 隨機漫步 4~16
  uint32_t m_epochs = 0;
  uint32_t m_rng = 0x12345678u;
  std::string m_lastEpochBytes;  // STUCK 用
  std::deque<Pending> m_pending;
  std::deque<uint8_t> m_wire;  // 待依 baud 灌入 RX 的位元組
  std::deque<uint8_t> m_tx;    // 已到 RX、可供 readBytes
};

}  // namespace Gnss
#endif
