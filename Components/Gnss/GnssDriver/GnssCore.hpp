// GnssCore.hpp - TEL0157 GNSS 驅動核心 (與 F´ 無關，可主機單測)
// schedIn＝rg100Hz；UART 讀取只在 poll 時刻（依 LockStage：100／1000／2000 ms）。不 sleep。
//
// Edward 2026-11-02 失鎖表（取代先前「只發事件、階段不分」的 C6/E4 解讀）：
//   TRACKING 100 ms 連續；失鎖 → REACQ_FAST → MED → SLOW → FIX_FAILED。
//   55 s 從「最後一次合格 GGA」起算（中斷真正開始的時刻）。LOSS_NO_GGA_S 偵測
//   耗時算進 REACQ_FAST：階段邊界＝自 last-good 起 15／25／55 s（即 FAST 15、
//   MED 再 10、SLOW 再 30）。任一段成功 → TRACKING。開機 ACQ 最多 300 s。
// Gate C6：不自己中止；fixOut 只接 tlm/sdLogger。E4：完全逾時＝進 FIX_FAILED。
//
// 假設：
//   A1. 開機 ACQ 逾時 → 一次 INIT_ACQ_TIMEOUT，進 FIX_FAILED（維持 READY、SLOW 輪詢；
//       之後鎖到可回 TRACKING）。不進 FAULT。BUS_ERROR／SELFTEST_FAIL 仍 → FAULT。
//   A2. FIX_FAILED 後仍以 SLOW 間隔輪詢，鎖定成功回 TRACKING（C6 不中止）。
//   A3. 失鎖條件：GGA 非合格 3D、或 LOSS_NO_GGA_S 內無新 GGA（含 UART 靜默）、或 STUCK_POLLS。
//   A4. 合格 3D＝fixType 3D 且 hdop<=HDOP_MAX；重鎖中 posValid=false、dataAge 自最後合格定位起算。
//   A5. NMEA 常為 1 Hz；空 poll 正常，不以單次空讀當失鎖。
//   A6. BUS_ERROR → FAULT；TRUNCATED／BAD_CHECKSUM 只計數。
//
// 輸出：新 GGA 才發定位並 ++seq；重鎖／靜默期間無新 GGA 時，最多 1 Hz 發降級
// （posValid=false、dataAge↑）供遙測。
//
// 讀取緩衝：kPollReadMax=2048。9600 8N1 → 960 B/s；2 s（含 GSV）≈1920 B，2048 留餘量。
// 仍為每 poll 一次 bus read。
#pragma once
#include <cstdint>
#include "GnssIfaces.hpp"
namespace Gnss {
struct GnssCoreParams {
  float hdopMax = 2.5f;
  float pollTracking_s = 0.1f;
  float pollReacqFast_s = 0.1f;
  float pollReacqMed_s = 1.0f;
  float pollReacqSlow_s = 2.0f;
  // 自 last-good 起算的階段配額（邊界＝15、25、55）
  float reacqFastDur_s = 15.f;
  float reacqMedDur_s = 10.f;
  float reacqSlowDur_s = 30.f;
  float initAcqTimeout_s = 300.f;
  float selftestTimeout_s = 5.f;   // SELFTEST：此時間內需任意合法 NMEA
  float lossNoGga_s = 2.0f;        // 無新 GGA 超過此秒 → 失鎖（含靜默；計入 FAST）
  uint32_t stuckPolls = 3;         // 連續多次 poll 收到相同 GGA → 失鎖
};

// 每 poll 一次 read 的上限（見檔頭：2 s@9600 含 GSV 的餘量）
constexpr uint32_t kPollReadMax = 2048;

class GnssCore {
 public:
  GnssCore(IGnssUart& uart, IGnssSink& sink) : m_uart(uart), m_sink(sink) {}
  void setParams(const GnssCoreParams& p) { m_p = p; }
  void start();
  void tick(double t_s);  // rg100Hz：每次呼叫；內部決定是否 poll
  DrvState state() const { return m_state; }
  LockStage lockStage() const { return m_lock; }
  uint32_t badChecksumCount() const { return m_badCk; }
  uint32_t truncatedCount() const { return m_trunc; }
  uint32_t pollsInStage() const { return m_pollsInStage; }  // 進本階段後的 poll 次數
  double lastGoodPos_s() const { return m_lastGoodPos_s; }
  bool feedSentence(const char* s, double t_s);
 private:
  void setState(DrvState s);
  void setLock(LockStage s, double t_s);
  float pollInterval_s() const;
  void doPoll(double t_s);
  void feedByte(uint8_t b, double t_s);
  void handleSentence(char* body, double t_s);
  void emitFix(double t_s, bool posValid);
  void maybeEmit(double t_s);
  bool qualityOk() const;
  bool detectLoss(double t_s) const;
  void onGoodFix(double t_s);
  void advanceReacq(double t_s);
  double outageAge_s(double t_s) const;
  IGnssUart& m_uart; IGnssSink& m_sink; GnssCoreParams m_p;
  DrvState m_state = DrvState::OFF;
  LockStage m_lock = LockStage::ACQ;
  double m_lastPoll_s = -1e9;
  double m_stageEnter_s = 0.0;
  double m_selftestEnter_s = 0.0;
  double m_acqEnter_s = 0.0;
  double m_lastNewGga_s = -1e9;
  double m_lastGoodPos_s = -1e9;
  double m_lastEmit_s = -1e9;
  uint32_t m_sameGgaPolls = 0, m_lastGgaHash = 0;
  uint32_t m_badCk = 0, m_trunc = 0, m_seq = 0;
  uint32_t m_pollsInStage = 0;
  bool m_gotValidSentence = false, m_newGga = false, m_haveFix = false;
  bool m_gotGgaThisPoll = false, m_fixFailedMsgSent = false;
  bool m_acqTimeoutSent = false;
  bool m_inLine = false;
  char m_line[96]; uint32_t m_len = 0;
  GnssFix m_fix{};
  uint8_t m_gsaMode = 1;
};
}  // namespace Gnss
