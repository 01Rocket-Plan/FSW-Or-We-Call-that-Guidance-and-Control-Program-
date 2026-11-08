#include "RfDriverCore.hpp"

namespace Components {

RfDriverCore::RfDriverCore(IRfBus& bus, IRfSink& sink, IRfTxQueue& txq)
    : m_bus(bus), m_sink(sink), m_txq(txq) {}

void RfDriverCore::setState(DrvState s) {
  if (s == m_state) return;
  const DrvState from = m_state;
  m_state = s;
  m_sink.onStateChange(from, s);
}

void RfDriverCore::start() {
  m_txInFlight = false;
  m_selftestTries = 0;
  m_consecTxTimeouts = 0;
  m_haveStart = false;
  setState(DrvState::INIT);
  m_sink.onComStatus(false);
}

void RfDriverCore::enterFault(RfEvent ev, uint32_t detail) {
  m_txInFlight = false;
  setState(DrvState::FAULT);
  m_sink.onEvent(ev, detail);
  m_sink.onEvent(RfEvent::FAULT, detail);
  m_sink.onComStatus(false);
}

void RfDriverCore::tick(double now_s) {
  if (!m_haveStart) {
    m_start_s = now_s;
    m_stateEnter_s = now_s;
    m_haveStart = true;
  }
  switch (m_state) {
    case DrvState::OFF:
      break;
    case DrvState::INIT:
      stepInit(now_s);
      break;
    case DrvState::SELFTEST:
      stepSelftest(now_s);
      break;
    case DrvState::READY:
      stepReady(now_s);
      break;
    case DrvState::FAULT:
      break;
    default:
      break;
  }
}

void RfDriverCore::stepInit(double now_s) {
  if (now_s - m_stateEnter_s > m_params.initDeadline_s) {
    enterFault(RfEvent::PHASE_TIMEOUT, 1);
    return;
  }
  if (!m_bus.init()) {
    enterFault(RfEvent::FAULT, 1);
    return;
  }
  m_sink.onEvent(RfEvent::INIT_OK, 0);
  m_stateEnter_s = now_s;
  setState(DrvState::SELFTEST);
}

void RfDriverCore::stepSelftest(double now_s) {
  if (now_s - m_stateEnter_s > m_params.selftestDeadline_s) {
    m_sink.onEvent(RfEvent::SELFTEST_FAIL, m_selftestTries);
    enterFault(RfEvent::PHASE_TIMEOUT, 2);
    return;
  }
  if (m_bus.selftestLoopback()) {
    m_sink.onEvent(RfEvent::SELFTEST_OK, 0);
    m_stateEnter_s = now_s;
    setState(DrvState::READY);
    m_sink.onEvent(RfEvent::READY, 0);
    m_sink.onComStatus(true);
  } else {
    ++m_selftestTries;
    if (m_selftestTries >= 3) {
      m_sink.onEvent(RfEvent::SELFTEST_FAIL, m_selftestTries);
      enterFault(RfEvent::FAULT, 2);
    }
  }
}

void RfDriverCore::stepReady(double now_s) {
  // overall ready-phase deadline only applies until first successful link ready;
  // once READY, use for optional re-init — here we just require bus.ready().
  if (!m_bus.ready()) {
    if (now_s - m_stateEnter_s > m_params.readyDeadline_s) {
      enterFault(RfEvent::PHASE_TIMEOUT, 3);
      return;
    }
    m_sink.onComStatus(false);
    return;
  }
  stepTx(now_s);
  stepRx();
}

void RfDriverCore::stepTx(double now_s) {
  if (m_txInFlight) {
    if (now_s - m_txStart_s > m_params.txTimeout_s) {
      m_txInFlight = false;
      ++m_txTimeoutCount;
      ++m_txDroppedCount;
      ++m_consecTxTimeouts;
      m_sink.onEvent(RfEvent::TX_TIMEOUT, m_txInFlightLen);
      m_sink.onComStatus(true);
      if (m_consecTxTimeouts >= m_params.txTimeoutLimit) {
        enterFault(RfEvent::FAULT, 4);
      }
      return;
    }
    // 下一 tick：視為硬體完成（真機由 IRQ／狀態暫存器推進；不 sleep）
    m_txInFlight = false;
    ++m_sentFrames;
    m_consecTxTimeouts = 0;
    m_sink.onComStatus(true);
    return;
  }

  m_frameLen = m_txq.popFrame(m_frameBuf, sizeof(m_frameBuf));
  if (m_frameLen == 0) return;

  const int32_t sent = m_bus.writeFrame(m_frameBuf, m_frameLen);
  if (sent <= 0) {
    ++m_txDroppedCount;
    m_sink.onEvent(RfEvent::TX_FAIL, m_frameLen);
    m_sink.onComStatus(true);
    return;
  }
  m_txInFlight = true;
  m_txInFlightLen = static_cast<uint32_t>(sent);
  m_txStart_s = now_s;
  m_sink.onComStatus(false);
}

void RfDriverCore::stepRx() {
  uint8_t buf[256];
  const int32_t n = m_bus.readBytes(buf, sizeof(buf));
  if (n > 0) m_sink.onUplink(buf, static_cast<uint32_t>(n));
}

}  // namespace Components
