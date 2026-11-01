#include "Components/EnvSensing/UsDht22/Dht22SimIo.hpp"
#include <cmath>

namespace Components {

void Dht22SimIo::encode(float t, float h, uint8_t b[5]) {
  int hr = static_cast<int>(std::lround(h * 10.0f));
  int tr = static_cast<int>(std::lround(std::fabs(t) * 10.0f));
  b[0] = static_cast<uint8_t>(hr >> 8);
  b[1] = static_cast<uint8_t>(hr);
  b[2] = static_cast<uint8_t>(((tr >> 8) & 0x7F) | (t < 0 ? 0x80 : 0));
  b[3] = static_cast<uint8_t>(tr);
  b[4] = static_cast<uint8_t>(b[0] + b[1] + b[2] + b[3]);
}

bool Dht22SimIo::start() {
  m_started = true;
  return true;
}

int Dht22SimIo::fetchHighs(uint16_t* out, int max) {
  if (!m_started || m_fault == Dht22SimFault::NO_RESPONSE) return -1;
  m_started = false;
  uint8_t b[5];
  if (m_fault == Dht22SimFault::STUCK && m_haveFrozen) {
    for (int i = 0; i < 5; i++) b[i] = m_frozen[i];
  } else {
    // 小幅抖動，避免靜置時看起來像卡值
    float jt = ((m_n++ % 3) - 1) * 0.1f;
    encode(m_fault == Dht22SimFault::OUT_OF_RANGE ? 150.0f : m_t + jt, m_h, b);
    for (int i = 0; i < 5; i++) m_frozen[i] = b[i];
    m_haveFrozen = true;
  }
  if (m_fault == Dht22SimFault::BAD_CHECKSUM) b[4] ^= 0x5A;
  int n = 0;
  auto push = [&](uint16_t v) { if (n < max) out[n++] = v; };
  push(30);  // 主機釋放後的短高電位
  push(80);  // DHT22 回應高電位
  int bits = (m_fault == Dht22SimFault::SHORT_FRAME) ? 30 : 40;
  for (int i = 0; i < bits; i++) push((b[i / 8] & (0x80 >> (i % 8))) ? 70 : 27);
  return n;
}

}  // namespace Components
