// SdCardSimDevice: in-memory SD media for Sim shell + host tests.
//
// 寫入速度模型（SIL 測試發現：SD 寫入沒有時間成本；見 SSS_Mission_Status #4／#5）：
//   每個寫入視窗（一次 advance() ＝ 一個 SdLogger schedIn tick）最多寫
//   writeBytesPerSec × windowUs / 1e6 位元組；額度用完 append() 回 false、狀態 BUSY，
//   SdLoggerCore::flush() 會把那筆留在佇列、下一 tick 再寫（不算 FAULT）。
//   額度可以透支一筆（一筆紀錄比一個視窗的額度大時不會卡死），透支量從下一個視窗扣回，
//   所以長期平均速度＝writeBytesPerSec。
//
// 預設數字與出處：
//   - writeBytesPerSec = 10 000 000 B/s：SD Association Speed Class「Class 10」與 UHS Speed Class
//     「U1」的最低連續寫入速度 10 MB/s（SD Physical Layer Simplified Specification 的 Speed Class
//     定義，性能單位 1 MB/s = 1 000 000 B/s；https://www.sdcard.org/developers/sd-standard-overview/speed-class/）。
//     Repo 只寫卡型 MICRO_SDHC_UHS_I_16GB（SdLogger.fpp），沒寫速度等級；Class 10／U1 是 UHS-I 卡的
//     最低標示等級，所以用它當保守下限。紀錄組可再改（Class 2/4/6 = 2/4/6 MB/s，U3 = 30 MB/s，
//     V6/V10/V30/V60/V90 = 6/10/30/60/90 MB/s）。
//   - windowUs = 10 000 µs：Deployments/DsCam、UsCam 拓樸把 sdLogger.schedIn 接在 rg100Hz。
//   - 單次寫入的忙碌時間：規格只給上限（SDHC 寫入逾時 250 ms，SD Physical Layer Simplified
//     Specification §4.6.2 Read, Write and Erase Timeout Conditions；CSD Version 2.0 的 R2W_FACTOR 欄位說明「host … should use 250 ms for write timeout」），沒有典型值 → 不編典型延遲；用 injectBusyWindows() 注入最壞情況
//     （250 ms ＝ 25 個 10 ms 視窗）做測試。真卡的典型延遲待冷測量。
//
// 不讀系統時間、不 sleep；時間只由 advance() 推進。
#ifndef RECORDER_SDCARDSIMDEVICE_HPP
#define RECORDER_SDCARDSIMDEVICE_HPP

#include "Components/Recorder/SdLogger/ISdMedia.hpp"
#include <cstdint>
#include <vector>

namespace Components {

enum class SdSimFaultKind : uint8_t {
  NONE = 0,
  FULL = 1,
  NACK = 2,
  WRITE_FAIL = 3,
  CARD_MISSING = 4,
};

class SdCardSimDevice : public ISdMedia {
 public:
  SdCardSimDevice();
  void setFault(SdSimFaultKind f) { m_fault = f; }
  SdSimFaultKind fault() const { return m_fault; }
  void setCapacity(size_t bytes) { m_capacity = bytes; }
  const std::vector<uint8_t>& image() const { return m_data; }
  void clear() { m_data.clear(); m_opened = false; m_status = SdMediaStatus::INIT; }

  //! SD Speed Class 對應的最低連續寫入速度（B/s；1 MB/s = 1 000 000 B/s）
  static constexpr uint32_t kClass2_Bps = 2000000u;
  static constexpr uint32_t kClass4_Bps = 4000000u;
  static constexpr uint32_t kClass6_Bps = 6000000u;
  static constexpr uint32_t kClass10_Bps = 10000000u;  //!< 也是 UHS U1、Video V10
  static constexpr uint32_t kU3_Bps = 30000000u;       //!< 也是 Video V30
  //! SDHC 寫入忙碌上限 250 ms（SD Physical Layer Simplified Spec §4.6.2）
  static constexpr uint32_t kSdhcWriteTimeout_us = 250000u;

  //! 寫入速度（B/s）；0 = 不限速（舊行為，只給需要的測試用）
  void setWriteBytesPerSec(uint32_t bps) { m_writeBps = bps; m_credit = windowBudget(); }
  uint32_t writeBytesPerSec() const { return m_writeBps; }
  //! 一個寫入視窗（一次 advance()）的長度 µs
  void setWindowUs(uint32_t us) { m_windowUs = us ? us : 1u; m_credit = windowBudget(); }
  uint32_t windowUs() const { return m_windowUs; }
  //! 一個視窗可寫的位元組數 = bps × windowUs / 1e6
  int64_t windowBudget() const {
    return static_cast<int64_t>(m_writeBps) * static_cast<int64_t>(m_windowUs) / 1000000;
  }
  //! 推進一個寫入視窗（SdLoggerSim 每個 schedIn 呼叫一次）
  void advance();
  //! 注入卡片忙碌 n 個視窗（例：SDHC 最壞 250 ms → 25 個 10 ms 視窗）
  void injectBusyWindows(uint32_t n) { m_busyWindows = n; }
  uint32_t busyRejects() const { return m_busyRejects; }

  bool open() override;
  bool append(const uint8_t* data, size_t len) override;
  bool sync() override;
  SdMediaStatus status() const override { return m_status; }
  size_t bytesWritten() const override { return m_data.size(); }

 private:
  std::vector<uint8_t> m_data;
  size_t m_capacity = 16u * 1024u * 1024u;  // 16 MiB sim default (UHS-I 16GB class scale-down)
  SdSimFaultKind m_fault = SdSimFaultKind::NONE;
  SdMediaStatus m_status = SdMediaStatus::INIT;
  bool m_opened = false;

  uint32_t m_writeBps = kClass10_Bps;
  uint32_t m_windowUs = 10000u;
  int64_t m_credit = static_cast<int64_t>(kClass10_Bps) * 10000 / 1000000;
  uint32_t m_busyWindows = 0;
  uint32_t m_busyRejects = 0;
};

}  // namespace Components
#endif
