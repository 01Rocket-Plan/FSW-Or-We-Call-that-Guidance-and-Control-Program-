// RfDriverCore：RF 數傳驅動核心（真機／Sim 共用）
// 握手：OFF → INIT → SELFTEST → READY／FAULT；各段截止 prm。
// 不 sleep、不讀系統時間；now_s 由呼叫端（F´ 時間服務）傳入。
// 連續 N 次 txTimeout → FAULT。不含金鑰或任務門檻。
// TX 完成（issue #60）：不再「下一 tick 就完成」。
//   - 有 TxDone IRQ 來源（真機，setTxDoneSource）：等 IRQ 才算送完。
//   - 沒有（Sim／模型）：送出後佔 ceil(ToA / tickPeriod_us) 個 tick 才算送完，ToA 照 LoraTimeOnAir。
//   - 兩種都一樣：超過「空中時間 + txTimeout_s」還沒完成 → TX_TIMEOUT。
#ifndef WIRELESS_RFDRIVERCORE_HPP
#define WIRELESS_RFDRIVERCORE_HPP

#include <cstdint>

#include "Components/Wireless/RfDriver/LoraTimeOnAir.hpp"

namespace Components {

enum class DrvState : uint8_t { OFF = 0, INIT = 1, SELFTEST = 2, READY = 3, FAULT = 4 };

enum class RfEvent : uint8_t {
  NONE = 0,
  INIT_OK = 1,
  SELFTEST_OK = 2,
  SELFTEST_FAIL = 3,
  READY = 4,
  PHASE_TIMEOUT = 5,
  TX_TIMEOUT = 6,
  TX_FAIL = 7,
  FAULT = 8,
};

struct RfParams {
  uint32_t channel = 0;
  int32_t power_dBm = 0;
  double txTimeout_s = 1.0;
  double initDeadline_s = 2.0;
  double selftestDeadline_s = 2.0;
  double readyDeadline_s = 5.0;  // INIT→READY 總截止（含握手）
  uint32_t txTimeoutLimit = 3;   // 連續 N 次 → FAULT

  // ---- LoRa SX1278 數據機（issue #60）。新欄位一律加在最後，不動舊欄位 ----
  // SF7／125 kHz／4:5 已由 Edward 定案；前導 8／CRC 開／顯式標頭仍「待 Edward 定」。
  // 注意：頻段仍在評估（433 MHz 法規問題），定案後可能換頻段／硬體
  uint8_t spreadingFactor = 7;                    // 6..12（SF6 必須隱式標頭）；定案 SF7
  LoraBw bandwidth = LoraBw::BW_125K;             // 7.8k..500k；定案 125 kHz
  LoraCr codingRate = LoraCr::CR_4_5;             // 4/5..4/8；定案 4/5
  uint16_t preambleLen = 8;                       // 前導碼符號數；待 Edward 定
  bool crcOn = true;                              // 酬載 CRC；待 Edward 定
  bool implicitHeader = false;                    // false＝顯式標頭；待 Edward 定
  LoraLdro lowDataRateOptimize = LoraLdro::AUTO;  // AUTO＝Tsym > 16 ms 開（Semtech §4.1.1.6）
  // 一次 tick() 的週期 µs。預設 10000（DsMcu 目前接在 rg100Hz）；正式 rate group 待定，改了要一起改
  uint32_t tickPeriod_us = 10000;
};

//! RfParams 的 LoRa 欄位 → LoraModemConfig（空中時間公式的輸入）
LoraModemConfig loraConfigFromRfParams(const RfParams& p);
//! RfParams 是否有效：LoRa 組合合法（SF／BW／CR 範圍、SF6 需隱式標頭）且 tickPeriod_us > 0
bool rfParamsValid(const RfParams& p);

//! 選用：傳輸層自己回報「這一幀送完了」。和 IRfBus 分開，不動 IRfBus。
//!   - LoRa 真機：SX1278 TxDone IRQ（RegIrqFlags.TxDone）
//!   - BLE（BleLinkCore）：最後一片被 BLE HAL 收下
//! 接了來源，Core 就不用 LoRa 空中時間判斷完成。
class IRfTxDoneSource {
 public:
  virtual ~IRfTxDoneSource() = default;
  //! 送完了就回 true 並清旗標（read-and-clear）
  virtual bool takeTxDone() = 0;
  //! 這一幀在傳輸層被放棄（例：BLE 送一半斷線）就回 true 並清旗標；Core 記 TX_FAIL／丟幀、不等逾時
  virtual bool takeTxAborted() { return false; }
  //! Core 判定 TX_TIMEOUT 時呼叫：傳輸層要放棄、清掉還沒送完的這一幀（例：BLE 清掉剩下的片），
  //! 之後才能再收新幀。預設什麼都不做（LoRa 真機：之後由驅動把 SX1278 切回 STANDBY）。
  virtual void abortTx() {}
  //! 這個傳輸層是不是 LoRa（SX1278）：
  //!   true  → 單包上限 255 B、逾時＝LoRa 空中時間 + txTimeout_s
  //!   false → 不套 LoRa 限制（例：BLE），逾時＝送出後 txTimeout_s（預期送完時間視為 0）
  virtual bool usesLoraAirtime() const { return true; }
};

class IRfBus {
 public:
  virtual ~IRfBus() = default;
  virtual bool init() = 0;
  virtual int32_t writeFrame(const uint8_t* data, uint32_t len) = 0;
  virtual int32_t readBytes(uint8_t* data, uint32_t maxLen) = 0;
  virtual bool selftestLoopback() = 0;
  virtual bool ready() = 0;
};

class IRfSink {
 public:
  virtual ~IRfSink() = default;
  virtual void onStateChange(DrvState from, DrvState to) = 0;
  virtual void onEvent(RfEvent ev, uint32_t detail) = 0;
  virtual void onUplink(const uint8_t* data, uint32_t len) = 0;
  virtual void onComStatus(bool ready) = 0;
};

class IRfTxQueue {
 public:
  virtual ~IRfTxQueue() = default;
  virtual uint32_t popFrame(uint8_t* buf, uint32_t maxLen) = 0;
};

class RfDriverCore {
 public:
  RfDriverCore(IRfBus& bus, IRfSink& sink, IRfTxQueue& txq);

  //! 參數無效（見 rfParamsValid）時 start() 直接進 FAULT（detail 5）
  void configure(const RfParams& p) { m_params = p; }
  //! 真機接 TxDone IRQ；nullptr（預設）＝用空中時間模型數 tick
  void setTxDoneSource(IRfTxDoneSource* src) { m_txDone = src; }
  void start();
  void tick(double now_s);

  DrvState state() const { return m_state; }
  uint32_t txTimeoutCount() const { return m_txTimeoutCount; }
  uint32_t txDroppedCount() const { return m_txDroppedCount; }
  uint32_t sentFrames() const { return m_sentFrames; }
  uint32_t consecutiveTxTimeouts() const { return m_consecTxTimeouts; }
  //! 正在送的這一包是否還在空中
  bool txInFlight() const { return m_txInFlight; }
  //! 最近一包的空中時間（µs）與佔幾個 tick（模型）
  uint64_t lastTxAirtimeUs() const { return m_txAirtime_us; }
  uint32_t lastTxAirtimeTicks() const { return m_txAirtimeTicks; }

 private:
  void setState(DrvState s);
  void stepInit(double now_s);
  void stepSelftest(double now_s);
  void stepReady(double now_s);
  void stepTx(double now_s);
  void stepRx();
  void enterFault(RfEvent ev, uint32_t detail);

  IRfBus& m_bus;
  IRfSink& m_sink;
  IRfTxQueue& m_txq;
  RfParams m_params{};

  DrvState m_state = DrvState::OFF;
  double m_stateEnter_s = 0.0;
  double m_start_s = 0.0;
  bool m_haveStart = false;

  bool m_txInFlight = false;
  double m_txStart_s = 0.0;
  uint32_t m_txInFlightLen = 0;
  uint64_t m_txAirtime_us = 0;
  uint32_t m_txAirtimeTicks = 0;
  uint32_t m_txTicksElapsed = 0;
  IRfTxDoneSource* m_txDone = nullptr;

  uint8_t m_frameBuf[512]{};
  uint32_t m_frameLen = 0;

  uint32_t m_txTimeoutCount = 0;
  uint32_t m_txDroppedCount = 0;
  uint32_t m_sentFrames = 0;
  uint32_t m_consecTxTimeouts = 0;
  uint32_t m_selftestTries = 0;
};

}  // namespace Components
#endif
