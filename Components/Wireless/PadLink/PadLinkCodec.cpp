#include "PadLinkCodec.hpp"

#include <cstring>

namespace PadLink {

namespace {

void put16(std::uint8_t* p, std::uint16_t v) {
  p[0] = static_cast<std::uint8_t>(v >> 8);
  p[1] = static_cast<std::uint8_t>(v & 0xFFu);
}
void put32(std::uint8_t* p, std::uint32_t v) {
  p[0] = static_cast<std::uint8_t>(v >> 24);
  p[1] = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
  p[2] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
  p[3] = static_cast<std::uint8_t>(v & 0xFFu);
}
std::uint16_t get16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}
std::uint32_t get32(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}

bool knownType(std::uint8_t t) {
  return t == static_cast<std::uint8_t>(PadMsgType::ROCKET_STATUS) ||
         t == static_cast<std::uint8_t>(PadMsgType::PAD_STATUS) ||
         t == static_cast<std::uint8_t>(PadMsgType::GROUND_CMD);
}

}  // namespace

std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t len) {
  std::uint16_t crc = 0xFFFFu;
  for (std::size_t i = 0; i < len; ++i) {
    crc = static_cast<std::uint16_t>(crc ^ (static_cast<std::uint16_t>(data[i]) << 8));
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000u) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021u)
                            : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::size_t encodeFrame(PadMsgType type, std::uint16_t seq, const std::uint8_t* payload,
                        std::size_t payloadLen, std::uint8_t* out, std::size_t outCap) {
  if (payloadLen > kMaxPayload || out == nullptr) return 0;
  if (payloadLen > 0 && payload == nullptr) return 0;
  const std::size_t total = kOverheadBytes + payloadLen;
  if (outCap < total) return 0;
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = static_cast<std::uint8_t>(type);
  put16(out + 3, seq);
  out[5] = static_cast<std::uint8_t>(payloadLen);
  if (payloadLen > 0) std::memcpy(out + kHeaderBytes, payload, payloadLen);
  const std::uint16_t crc = crc16Ccitt(out + 2, 4 + payloadLen);
  put16(out + kHeaderBytes + payloadLen, crc);
  return total;
}

bool decodeFrame(const std::uint8_t* data, std::size_t len, DecodedFrame& out) {
  if (data == nullptr || len < kOverheadBytes) return false;
  if (data[0] != kSync0 || data[1] != kSync1) return false;
  if (!knownType(data[2])) return false;
  const std::size_t plen = data[5];
  if (plen > kMaxPayload || len < kOverheadBytes + plen) return false;
  const std::uint16_t want = get16(data + kHeaderBytes + plen);
  if (crc16Ccitt(data + 2, 4 + plen) != want) return false;
  out.type = static_cast<PadMsgType>(data[2]);
  out.seq = get16(data + 3);
  out.len = static_cast<std::uint8_t>(plen);
  if (plen > 0) std::memcpy(out.payload, data + kHeaderBytes, plen);
  return true;
}

bool StreamDeframer::push(std::uint8_t byte, DecodedFrame& out) {
  if (m_pos == 0) {
    if (byte == kSync0) m_buf[m_pos++] = byte;
    return false;
  }
  if (m_pos == 1) {
    if (byte == kSync1) {
      m_buf[m_pos++] = byte;
    } else {
      m_pos = (byte == kSync0) ? 1 : 0;
    }
    return false;
  }
  m_buf[m_pos++] = byte;
  if (m_pos == kHeaderBytes) {
    if (m_buf[5] > kMaxPayload || !knownType(m_buf[2])) {
      ++m_lenErrors;
      m_pos = 0;
    }
    return false;
  }
  if (m_pos < kHeaderBytes) return false;
  const std::size_t need = kOverheadBytes + m_buf[5];
  if (m_pos < need) return false;
  const bool ok = decodeFrame(m_buf, need, out);
  if (!ok) ++m_crcErrors;
  m_pos = 0;
  return ok;
}

// ---------------------------------------------------------------------------

std::size_t packRocketStatus(const RocketStatusMsg& m, std::uint8_t* out, std::size_t cap) {
  if (out == nullptr || cap < kRocketStatusBytes) return 0;
  out[0] = m.flightMode;
  put16(out + 1, m.bitFlags);
  out[3] = m.ready ? 1u : 0u;
  put16(out + 4, m.battV_mV);
  out[6] = m.bootCount;
  return kRocketStatusBytes;
}

bool unpackRocketStatus(const std::uint8_t* p, std::size_t len, RocketStatusMsg& m) {
  if (p == nullptr || len != kRocketStatusBytes) return false;
  if (p[3] > 1u) return false;
  m.flightMode = p[0];
  m.bitFlags = get16(p + 1);
  m.ready = (p[3] == 1u);
  m.battV_mV = get16(p + 4);
  m.bootCount = p[6];
  return true;
}

std::size_t packPadStatus(const PadStatusMsg& m, std::uint8_t* out, std::size_t cap) {
  if (out == nullptr || cap < kPadStatusBytes) return 0;
  out[0] = m.padState;
  out[1] = m.lastReason;
  put16(out + 2, m.interlocks);
  out[4] = m.rocketMode;
  put16(out + 5, m.rocketBit);
  put16(out + 7, m.tilt_cdeg);
  put16(out + 9, m.padV_mV);
  put16(out + 11, m.countdown_ds);
  put32(out + 13, m.lastCmdCounter);
  out[17] = m.lastCmdResult;
  out[18] = m.bootId;
  return kPadStatusBytes;
}

bool unpackPadStatus(const std::uint8_t* p, std::size_t len, PadStatusMsg& m) {
  if (p == nullptr || len != kPadStatusBytes) return false;
  m.padState = p[0];
  m.lastReason = p[1];
  m.interlocks = get16(p + 2);
  m.rocketMode = p[4];
  m.rocketBit = get16(p + 5);
  m.tilt_cdeg = get16(p + 7);
  m.padV_mV = get16(p + 9);
  m.countdown_ds = get16(p + 11);
  m.lastCmdCounter = get32(p + 13);
  m.lastCmdResult = p[17];
  m.bootId = p[18];
  return true;
}

std::size_t packGroundCmd(const GroundCmdMsg& m, std::uint8_t* out, std::size_t cap) {
  if (out == nullptr || cap < kGroundCmdBytes) return 0;
  out[0] = static_cast<std::uint8_t>(m.op);
  put32(out + 1, m.counter);
  out[5] = m.bootId;
  std::memcpy(out + 6, m.tag, kCmdTagBytes);
  return kGroundCmdBytes;
}

bool unpackGroundCmd(const std::uint8_t* p, std::size_t len, GroundCmdMsg& m) {
  if (p == nullptr || len != kGroundCmdBytes) return false;
  if (p[0] < static_cast<std::uint8_t>(PadCmdOp::ARM) ||
      p[0] > static_cast<std::uint8_t>(PadCmdOp::HEARTBEAT)) {
    return false;
  }
  m.op = static_cast<PadCmdOp>(p[0]);
  m.counter = get32(p + 1);
  m.bootId = p[5];
  std::memcpy(m.tag, p + 6, kCmdTagBytes);
  return true;
}

}  // namespace PadLink
