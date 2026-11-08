#include "ComQueueCore.hpp"

namespace Components {

void ComQueueCore::Ring::reset() {
  head = 0;
  tail = 0;
  count = 0;
  hwm = 0;
}

bool ComQueueCore::Ring::pushDropNewest(const ComFrame& f) {
  if (count >= depth) {
    (void)f;
    return true;
  }
  buf[tail] = f;
  tail = static_cast<uint8_t>((tail + 1) % kMaxDepth);
  ++count;
  if (count > hwm) hwm = count;
  return false;
}

bool ComQueueCore::Ring::replaceApid(const ComFrame& f) {
  for (uint8_t i = 0; i < count; ++i) {
    ComFrame& slot = buf[(head + i) % kMaxDepth];
    if (slot.apid == f.apid) {
      slot = f;
      return true;
    }
  }
  return false;
}

bool ComQueueCore::Ring::pop(ComFrame& out) {
  if (count == 0) return false;
  out = buf[head];
  head = static_cast<uint8_t>((head + 1) % kMaxDepth);
  --count;
  return true;
}

ComQueueCore::ComQueueCore() {
  for (uint8_t i = 0; i < static_cast<uint8_t>(ComPriority::NUM); ++i) {
    m_ring[i].reset();
    m_ring[i].depth = kMaxDepth;
  }
}

bool ComQueueCore::setDepth(ComPriority p, uint8_t depth) {
  const uint8_t idx = static_cast<uint8_t>(p);
  if (idx >= static_cast<uint8_t>(ComPriority::NUM)) return false;
  if (depth < 1 || depth > kMaxDepth) return false;
  Ring& r = m_ring[idx];
  r.reset();
  r.depth = depth;
  return true;
}

bool ComQueueCore::setMaxPktsPerRun(uint8_t n) {
  if (n < 1) return false;
  m_maxPktsPerRun = n;
  return true;
}

bool ComQueueCore::setLatestOnly(ComPriority p, uint16_t apid) {
  if (static_cast<uint8_t>(p) >= static_cast<uint8_t>(ComPriority::NUM)) return false;
  if (isLatestOnly(p, apid)) return true;
  if (m_latestOnlyN >= kMaxLatestOnly) return false;
  m_latestOnly[m_latestOnlyN].pri = p;
  m_latestOnly[m_latestOnlyN].apid = apid;
  ++m_latestOnlyN;
  return true;
}

bool ComQueueCore::isLatestOnly(ComPriority p, uint16_t apid) const {
  for (uint8_t i = 0; i < m_latestOnlyN; ++i) {
    if (m_latestOnly[i].pri == p && m_latestOnly[i].apid == apid) return true;
  }
  return false;
}

void ComQueueCore::noteDrop(ComPriority p) {
  const uint8_t idx = static_cast<uint8_t>(p);
  ++m_dropTotal[idx];
  ++m_dropSinceEmit[idx];
  m_pendingDropEvent = true;
  m_pendingDropPri = p;
  m_event = ComQueueEvent::QUEUE_FULL_DROP;
  if (!m_hasEmittedDrop ||
      (m_tickCount - m_lastDropEmitTick) >= kDropEventPeriodTicks) {
    flushDropEvent();
  }
}

void ComQueueCore::flushDropEvent() {
  if (!m_pendingDropEvent) return;
  const uint8_t idx = static_cast<uint8_t>(m_pendingDropPri);
  const uint32_t n = m_dropSinceEmit[idx];
  uint8_t best = idx;
  uint32_t bestN = n;
  for (uint8_t i = 0; i < static_cast<uint8_t>(ComPriority::NUM); ++i) {
    if (m_dropSinceEmit[i] > bestN) {
      bestN = m_dropSinceEmit[i];
      best = i;
    }
  }
  if (bestN == 0) {
    m_pendingDropEvent = false;
    return;
  }
  if (m_events != nullptr) {
    m_events->onQueueFullDrop(static_cast<ComPriority>(best), bestN);
  }
  for (uint8_t i = 0; i < static_cast<uint8_t>(ComPriority::NUM); ++i) {
    m_dropSinceEmit[i] = 0;
  }
  m_pendingDropEvent = false;
  m_hasEmittedDrop = true;
  m_lastDropEmitTick = m_tickCount;
  m_event = ComQueueEvent::QUEUE_FULL_DROP;
}

bool ComQueueCore::enqueue(ComPriority p, const ComFrame& f) {
  const uint8_t idx = static_cast<uint8_t>(p);
  if (idx >= static_cast<uint8_t>(ComPriority::NUM)) return false;
  Ring& r = m_ring[idx];
  // 只留最新：已有同 apid 未送出 → 原位覆蓋，不佔新格、不算丟包（GNSS fix 不會塞滿 P0 擠掉事件）
  if (isLatestOnly(p, f.apid) && r.replaceApid(f)) {
    ++m_coalesce[idx];
    return true;
  }
  // 全層滿丟最新（P0 是關鍵事件，先發生的不能被後來的擠掉）
  const bool dropped = r.pushDropNewest(f);
  if (dropped) {
    noteDrop(p);
    updateHighWater();
    return false;
  }
  updateHighWater();
  return true;
}

bool ComQueueCore::dequeue(ComFrame& out) {
  for (uint8_t i = 0; i < static_cast<uint8_t>(ComPriority::NUM); ++i) {
    if (m_ring[i].pop(out)) {
      updateHighWater();
      return true;
    }
  }
  return false;
}

bool ComQueueCore::nextFrame(bool linkBusy, ComFrame& out) {
  if (linkBusy) return false;
  return dequeue(out);
}

uint8_t ComQueueCore::runEmit(bool linkBusy, ComFrame* out, uint8_t outCap) {
  tick();
  if (linkBusy || out == nullptr || outCap == 0) return 0;
  const uint8_t lim = m_maxPktsPerRun < outCap ? m_maxPktsPerRun : outCap;
  uint8_t n = 0;
  while (n < lim) {
    // busy-hold：本 run 假設狀態在呼叫時已就緒；若之後變 busy 由 F´ 殼下次 run 再查
    if (!dequeue(out[n])) break;
    ++n;
  }
  return n;
}

void ComQueueCore::tick() {
  ++m_tickCount;
  if (m_pendingDropEvent &&
      (m_tickCount - m_lastDropEmitTick) >= kDropEventPeriodTicks) {
    flushDropEvent();
  }
  updateHighWater();
}

void ComQueueCore::updateHighWater() {
  bool anyHigh = false;
  bool allLow = true;
  for (uint8_t i = 0; i < static_cast<uint8_t>(ComPriority::NUM); ++i) {
    const Ring& r = m_ring[i];
    if (r.depth == 0) continue;
    if (static_cast<uint16_t>(r.count) * 4u >= static_cast<uint16_t>(r.depth) * 3u) {
      anyHigh = true;
    }
    if (static_cast<uint16_t>(r.count) * 2u >= static_cast<uint16_t>(r.depth)) {
      allLow = false;
    }
  }
  if (anyHigh) {
    if (!m_highWater) {
      m_highWater = true;
      m_event = ComQueueEvent::HIGH_WATER;
    }
  } else if (allLow) {
    m_highWater = false;
  }
}

ComQueueEvent ComQueueCore::takeEvent() {
  const ComQueueEvent e = m_event;
  m_event = ComQueueEvent::NONE;
  return e;
}

uint8_t ComQueueCore::depth(ComPriority p) const {
  const uint8_t idx = static_cast<uint8_t>(p);
  if (idx >= static_cast<uint8_t>(ComPriority::NUM)) return 0;
  return m_ring[idx].depth;
}

uint8_t ComQueueCore::count(ComPriority p) const {
  const uint8_t idx = static_cast<uint8_t>(p);
  if (idx >= static_cast<uint8_t>(ComPriority::NUM)) return 0;
  return m_ring[idx].count;
}

uint8_t ComQueueCore::hwm(ComPriority p) const {
  const uint8_t idx = static_cast<uint8_t>(p);
  if (idx >= static_cast<uint8_t>(ComPriority::NUM)) return 0;
  return m_ring[idx].hwm;
}

uint32_t ComQueueCore::dropCount(ComPriority p) const {
  const uint8_t idx = static_cast<uint8_t>(p);
  if (idx >= static_cast<uint8_t>(ComPriority::NUM)) return 0;
  return m_dropTotal[idx];
}

uint32_t ComQueueCore::dropCountTotal() const {
  uint32_t s = 0;
  for (uint8_t i = 0; i < static_cast<uint8_t>(ComPriority::NUM); ++i) s += m_dropTotal[i];
  return s;
}

uint32_t ComQueueCore::coalesceCount(ComPriority p) const {
  const uint8_t idx = static_cast<uint8_t>(p);
  if (idx >= static_cast<uint8_t>(ComPriority::NUM)) return 0;
  return m_coalesce[idx];
}

uint32_t ComQueueCore::queuedTotal() const {
  uint32_t s = 0;
  for (uint8_t i = 0; i < static_cast<uint8_t>(ComPriority::NUM); ++i) s += m_ring[i].count;
  return s;
}

}  // namespace Components
