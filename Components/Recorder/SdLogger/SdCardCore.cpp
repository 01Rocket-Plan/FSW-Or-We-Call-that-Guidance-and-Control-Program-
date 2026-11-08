#include "Components/Recorder/SdLogger/SdCardCore.hpp"
#include <cstring>

namespace Components {

SdCardCore::SdCardCore(ISdMedia& media) : m_media(media) {}

bool SdCardCore::init() {
  if (!m_media.open()) {
    ++m_errors;
    return false;
  }
  return true;
}

bool SdCardCore::writeRecord(const SdRecordHeader& hdr, const uint8_t* payload, uint16_t payloadLen) {
  if (payloadLen > kMaxPayload) {
    ++m_errors;
    return false;
  }
  if (m_poolInUse >= m_poolSize) {
    // Pool exhausted: treat as transient busy (caller may retry after sync drain)
    ++m_errors;
    return false;
  }
  ++m_poolInUse;
  if (m_poolInUse > m_poolHwm) m_poolHwm = m_poolInUse;

  uint8_t frame[8 + 2 + 4 + 4 + 8 + 2 + kMaxPayload];
  size_t off = 0;
  const uint32_t magic = kMagic;
  std::memcpy(frame + off, &magic, 4); off += 4;
  std::memcpy(frame + off, &hdr.channelId, 2); off += 2;
  std::memcpy(frame + off, &hdr.loggerSeq, 4); off += 4;
  std::memcpy(frame + off, &hdr.producerSeq, 4); off += 4;
  std::memcpy(frame + off, &hdr.t_s, 8); off += 8;
  std::memcpy(frame + off, &payloadLen, 2); off += 2;
  if (payloadLen && payload) {
    std::memcpy(frame + off, payload, payloadLen);
    off += payloadLen;
  }
  const bool ok = m_media.append(frame, off);
  // Buffer returns to pool after append attempt (sync path is separate)
  if (m_poolInUse) --m_poolInUse;
  if (!ok) {
    ++m_errors;
    return false;
  }
  ++m_writes;
  return true;
}

bool SdCardCore::sync() {
  if (!m_media.sync()) {
    ++m_errors;
    return false;
  }
  return true;
}

}  // namespace Components
