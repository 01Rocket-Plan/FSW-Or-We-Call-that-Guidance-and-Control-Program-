#include "TlmSendCore.hpp"

namespace Components {

TlmChannel* TlmSendCore::find(uint16_t apid) {
  for (size_t i = 0; i < m_numCh; ++i) {
    if (m_chans[i].apid == apid) return &m_chans[i];
  }
  return nullptr;
}

bool TlmSendCore::addChannel(uint16_t apid, uint32_t rateDiv) {
  if (m_numCh >= kMaxChannels) return false;
  if (find(apid) != nullptr) return false;
  if (rateDiv == 0) rateDiv = 1;
  TlmChannel& c = m_chans[m_numCh++];
  c.apid = apid;
  c.rateDiv = rateDiv;
  c.enabled = true;
  c.nextTick = 0;
  c.seq = 0;
  c.lastTs = 0.0;
  c.haveLastTs = false;
  return true;
}

bool TlmSendCore::setEnabled(uint16_t apid, bool en) {
  TlmChannel* c = find(apid);
  if (c == nullptr) return false;
  c->enabled = en;
  return true;
}

bool TlmSendCore::setRateDiv(uint16_t apid, uint32_t rateDiv) {
  TlmChannel* c = find(apid);
  if (c == nullptr) return false;
  c->rateDiv = (rateDiv == 0) ? 1 : rateDiv;
  return true;
}

uint32_t TlmSendCore::seqOf(uint16_t apid) const {
  for (size_t i = 0; i < m_numCh; ++i) {
    if (m_chans[i].apid == apid) return m_chans[i].seq;
  }
  return 0u;
}

void TlmSendCore::tick() {
  const uint32_t nowTick = m_tickCount;
  for (size_t i = 0; i < m_numCh; ++i) {
    TlmChannel& c = m_chans[i];
    if (!c.enabled) continue;
    if (nowTick < c.nextTick) continue;

    TlmPacket pkt{};
    uint16_t len = 0;
    if (m_provider != nullptr) {
      len = m_provider->sample(c.apid, pkt.payload, sizeof(pkt.payload));
    }
    if (len == 0) {
      c.nextTick = nowTick + c.rateDiv;
      continue;
    }
    pkt.payloadLen = len;
    pkt.apid = c.apid;
    pkt.type = TlmPktType::TLM;
    pkt.src = m_src;
    pkt.bootCount = m_boot;

    const double ts = (m_time != nullptr) ? m_time->now_s() : 0.0;
    if (c.haveLastTs && ts < c.lastTs) {
      ++m_timeNonMonoCount;
      if (m_events != nullptr) {
        m_events->onTimeNonMonotonic(c.apid, c.lastTs, ts);
      }
      // 仍照常送出實際 t_s（不夾時間）
    }
    c.lastTs = ts;
    c.haveLastTs = true;

    pkt.hdr.t_s = ts;
    pkt.hdr.seq = c.seq;
    pkt.hdr.valid = true;

    if (m_sink != nullptr) m_sink->onPacket(pkt);
    c.seq++;
    c.nextTick = nowTick + c.rateDiv;
  }
  m_tickCount++;
}

}  // namespace Components
