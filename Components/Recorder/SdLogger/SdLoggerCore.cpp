#include "Components/Recorder/SdLogger/SdLoggerCore.hpp"

namespace Components {

SdLoggerCore::SdLoggerCore(SdCardCore& card, ISdLoggerSink& sink) : m_card(card), m_sink(sink) {}

void SdLoggerCore::setState(LoggerDrvState s, uint8_t faultReason) {
  if (m_state == s) return;
  m_state = s;
  if (s == LoggerDrvState::READY) m_sink.onEnterReady();
  if (s == LoggerDrvState::FAULT) m_sink.onEnterFault(faultReason);
}

bool SdLoggerCore::start() {
  setState(LoggerDrvState::INIT);
  setState(LoggerDrvState::SELFTEST);
  if (!m_card.init()) {
    m_sink.onSdError(m_card.status(), 1);
    setState(LoggerDrvState::FAULT, 1);
    return false;
  }
  m_running = true;
  setState(LoggerDrvState::READY);
  return true;
}

bool SdLoggerCore::checkSeq(uint16_t channelId, uint32_t producerSeq) {
  const size_t idx = channelId < 32 ? channelId : 0;
  if (!m_haveSeq[idx]) {
    m_haveSeq[idx] = true;
    m_expectSeq[idx] = producerSeq + 1;
    return true;
  }
  const uint32_t expected = m_expectSeq[idx];
  if (producerSeq != expected) {
    m_sink.onSeqGap(expected, producerSeq);
    m_expectSeq[idx] = producerSeq + 1;
    return true;  // still accept; gap is signaled
  }
  m_expectSeq[idx] = producerSeq + 1;
  return true;
}

bool SdLoggerCore::checkTime(double t_s) {
  if (m_haveTime && t_s < m_lastT_s) {
    m_sink.onTimeNonMonotonic(m_lastT_s, t_s);
    // still accept per policy (detect/emit); do not block
  }
  m_haveTime = true;
  m_lastT_s = t_s;
  return true;
}

bool SdLoggerCore::ingest(const PackedRecord& rec) {
  if (!m_running || m_state != LoggerDrvState::READY) return false;
  checkSeq(rec.channelId, rec.producerSeq);
  checkTime(rec.t_s);
  m_lastProducerSeq = rec.producerSeq;

  if (m_q.size() >= m_queueDepth) {
    ++m_drops;
    m_sink.onQueueFullDrop(m_drops, static_cast<uint32_t>(m_q.size()));
    return false;  // drop newest
  }
  m_q.push_back(rec);
  if (m_q.size() > m_queueHwm) m_queueHwm = static_cast<uint32_t>(m_q.size());
  return true;
}

void SdLoggerCore::flush() {
  while (!m_q.empty()) {
    const PackedRecord& rec = m_q.front();
    SdRecordHeader hdr;
    hdr.channelId = rec.channelId;
    hdr.loggerSeq = m_loggerSeq;
    hdr.producerSeq = rec.producerSeq;
    hdr.t_s = rec.t_s;
    if (!m_card.writeRecord(hdr, rec.bytes, rec.len)) {
      m_sink.onSdError(m_card.status(), 2);
      if (m_card.status() == SdMediaStatus::CARD_MISSING || m_card.status() == SdMediaStatus::IO_ERROR) {
        setState(LoggerDrvState::FAULT, 2);
      }
      break;  // leave remaining queued; never block callers
    }
    ++m_loggerSeq;
    m_q.pop_front();
  }
  (void)m_card.sync();
}

}  // namespace Components
