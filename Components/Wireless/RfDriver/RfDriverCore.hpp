// RfDriverCore：RF 數傳驅動核心（真機／Sim 共用）
// 握手：OFF → INIT → SELFTEST → READY／FAULT；各段截止 prm。
// 不 sleep、不讀系統時間；now_s 由呼叫端（F´ 時間服務）傳入。
// 連續 N 次 txTimeout → FAULT。不含金鑰或任務門檻。
#ifndef WIRELESS_RFDRIVERCORE_HPP
#define WIRELESS_RFDRIVERCORE_HPP

#include <cstdint>

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

  void configure(const RfParams& p) { m_params = p; }
  void start();
  void tick(double now_s);

  DrvState state() const { return m_state; }
  uint32_t txTimeoutCount() const { return m_txTimeoutCount; }
  uint32_t txDroppedCount() const { return m_txDroppedCount; }
  uint32_t sentFrames() const { return m_sentFrames; }
  uint32_t consecutiveTxTimeouts() const { return m_consecTxTimeouts; }

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
