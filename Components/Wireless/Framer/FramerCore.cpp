#include "FramerCore.hpp"
#include <cstring>

namespace Components {

namespace {
inline void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}
inline void putU32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<uint8_t>(v & 0xFF);
}
inline void putU64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    p[i] = static_cast<uint8_t>((v >> (56 - 8 * i)) & 0xFF);
  }
}
inline uint64_t f64ToBits(double d) {
  uint64_t u = 0;
  static_assert(sizeof(double) == sizeof(uint64_t), "F64 layout");
  std::memcpy(&u, &d, sizeof(u));
  return u;
}
}  // namespace

uint32_t FramerCore::crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (int b = 0; b < 8; ++b) {
      const uint32_t mask = 0u - (crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

void FramerCore::setFrameMax(size_t frameMax_B) { m_frameMax = frameMax_B; }

void FramerCore::emitError(FramerError e, uint32_t detail) {
  if (m_events != nullptr) m_events->onFramerError(e, detail);
}

FrameResult FramerCore::build(const FrameRequest& req, uint8_t* out, size_t outCap) {
  FrameResult r{};
  if (req.payload == nullptr && req.payloadLen != 0) {
    emitError(FramerError::NULL_PAYLOAD, req.payloadLen);
    return r;
  }
  if ((req.apid & 0xF800u) != 0) {
    emitError(FramerError::BAD_APID, req.apid);
    return r;
  }
  const size_t total =
      FRAMER_HEADER_BYTES + static_cast<size_t>(req.payloadLen) + FRAMER_TRAILER_CRC;
  if (m_frameMax != 0 && total > m_frameMax) {
    emitError(FramerError::PAYLOAD_TOO_LARGE, static_cast<uint32_t>(total));
    return r;
  }
  if (out == nullptr || outCap < total) {
    emitError(FramerError::BUFFER_TOO_SMALL, static_cast<uint32_t>(outCap));
    return r;
  }

  putU32(out + FRAMER_OFF_SYNC, FRAMER_SYNC);
  putU32(out + FRAMER_OFF_LEN, static_cast<uint32_t>(req.payloadLen));
  out[FRAMER_OFF_VER_TYPE] = static_cast<uint8_t>(static_cast<uint8_t>(req.type) & 0x1Fu);
  out[FRAMER_OFF_SOURCE] = static_cast<uint8_t>(req.source);
  putU16(out + FRAMER_OFF_APID, static_cast<uint16_t>(req.apid & 0x07FFu));
  putU32(out + FRAMER_OFF_SEQ, req.seq);
  out[FRAMER_OFF_BOOT] = req.bootCount;
  putU64(out + FRAMER_OFF_TS, f64ToBits(req.t_s));
  out[FRAMER_OFF_VALID] = req.valid ? 1u : 0u;
  for (uint16_t i = 0; i < req.payloadLen; ++i) {
    out[FRAMER_OFF_PAYLOAD + i] = req.payload[i];
  }
  const size_t crcStart = FRAMER_OFF_LEN;
  const size_t crcLen = total - FRAMER_TRAILER_CRC - crcStart;
  const uint32_t c = crc32(out + crcStart, crcLen);
  putU32(out + total - FRAMER_TRAILER_CRC, c);

  r.data = out;
  r.len = total;
  r.ok = true;
  return r;
}

}  // namespace Components
