#include "RfDriverCore.hpp"

namespace Components {

LoraModemConfig loraConfigFromRfParams(const RfParams& p) {
  LoraModemConfig c{};
  c.sf = p.spreadingFactor;
  c.bw_Hz = loraBwHz(p.bandwidth);  // 無效編碼 → 0 → loraConfigValid 為 false
  c.cr = static_cast<uint8_t>(p.codingRate);
  c.preambleLen = p.preambleLen;
  c.explicitHeader = !p.implicitHeader;
  c.crcOn = p.crcOn;
  c.ldro = p.lowDataRateOptimize;
  return c;
}

bool rfParamsValid(const RfParams& p) {
  return (p.tickPeriod_us > 0u) && loraConfigValid(loraConfigFromRfParams(p));
}

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
  m_txTicksElapsed = 0;
  m_selftestTries = 0;
  m_consecTxTimeouts = 0;
  m_haveStart = false;
  setState(DrvState::INIT);
  m_sink.onComStatus(false);
  if (!rfParamsValid(m_params)) {
    // LoRa 參數組合不合法（例：SF6＋顯式標頭、SF 超出 6..12）：不能算空中時間，也不該讓硬體這樣設
    enterFault(RfEvent::FAULT, 5);
  }
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
  // 空中時間模型：每個 tick 都算（無論 bus.ready()），因為無線電照樣在發
  if (m_txInFlight && m_txTicksElapsed < 0xFFFFFFFFu) ++m_txTicksElapsed;
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
    const bool aborted = (m_txDone != nullptr) && m_txDone->takeTxAborted();
    // 完成條件：有 TxDone 來源就等它；沒有就照空中時間數 tick（ceil(ToA / tickPeriod_us)）
    const bool done = !aborted && ((m_txDone != nullptr) ? m_txDone->takeTxDone()
                                                         : (m_txTicksElapsed >= m_txAirtimeTicks));
    if (aborted) {
      // 傳輸層放棄這一幀（例：BLE 送一半斷線）：記丟幀，不算逾時；傳輸層已空出來
      m_txInFlight = false;
      ++m_txDroppedCount;
      m_sink.onEvent(RfEvent::TX_FAIL, m_txInFlightLen);
      m_sink.onComStatus(true);
    } else if (done) {
      m_txInFlight = false;
      ++m_sentFrames;
      m_consecTxTimeouts = 0;
      m_sink.onComStatus(true);
    } else {
      // 逾時：超過「預期送完時間（LoRa 空中時間；非 LoRa 為 0）+ txTimeout_s」還沒完成
      const double limit_s = static_cast<double>(m_txAirtime_us) * 1.0e-6 + m_params.txTimeout_s;
      if (now_s - m_txStart_s > limit_s) {
        m_txInFlight = false;
        // 通知傳輸層放棄這一幀：否則像 BLE 這種「上一幀沒清掉就不收新幀」的傳輸層會一直卡住，
        // 下一幀出不了佇列、也就不會再逾時，連續 txTimeoutLimit 次到不了 FAULT
        if (m_txDone != nullptr) m_txDone->abortTx();
        ++m_txTimeoutCount;
        ++m_txDroppedCount;
        ++m_consecTxTimeouts;
        m_sink.onEvent(RfEvent::TX_TIMEOUT, m_txInFlightLen);
        m_sink.onComStatus(true);
        if (m_consecTxTimeouts >= m_params.txTimeoutLimit) {
          enterFault(RfEvent::FAULT, 4);
        }
      }
      return;
    }
    // 傳輸層已空出來：同一 tick 就可以送下一包（往下走）
  }

  m_frameLen = m_txq.popFrame(m_frameBuf, sizeof(m_frameBuf));
  if (m_frameLen == 0) return;

  const bool lora = (m_txDone == nullptr) || m_txDone->usesLoraAirtime();
  const LoraModemConfig modem = loraConfigFromRfParams(m_params);
  if (lora && loraTimeOnAirUs(modem, m_frameLen) == 0u) {
    // SX1278 單包 PL 只能 1..255（RegPayloadLength 8 bit）：發不出去，不交給硬體
    ++m_txDroppedCount;
    m_sink.onEvent(RfEvent::TX_FAIL, m_frameLen);
    m_sink.onComStatus(true);
    return;
  }

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
  // 非 LoRa 傳輸層（BLE）：預期送完時間 0，逾時＝送出後 txTimeout_s
  m_txAirtime_us = lora ? loraTimeOnAirUs(modem, m_txInFlightLen) : 0u;
  m_txAirtimeTicks = lora ? loraAirtimeTicks(modem, m_txInFlightLen, m_params.tickPeriod_us) : 0u;
  m_txTicksElapsed = 0;
  m_sink.onComStatus(false);
}

void RfDriverCore::stepRx() {
  uint8_t buf[256];
  const int32_t n = m_bus.readBytes(buf, sizeof(buf));
  if (n > 0) m_sink.onUplink(buf, static_cast<uint32_t>(n));
}

}  // namespace Components
