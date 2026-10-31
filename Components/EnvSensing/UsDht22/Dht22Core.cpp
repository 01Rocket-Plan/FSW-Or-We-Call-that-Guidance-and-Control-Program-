#include "Components/EnvSensing/UsDht22/Dht22Core.hpp"

namespace EnvSensing {
namespace Dht22Decode {

bool bitsFromHighs(const uint16_t* h, int n, uint16_t thr, uint8_t out[5]) {
  if (n < 40) return false;
  const uint16_t* b = h + (n - 40);  // 前面是主機釋放與 80 µs 回應
  for (int i = 0; i < 5; i++) out[i] = 0;
  for (int i = 0; i < 40; i++) {
    if (b[i] < 10 || b[i] > 120) return false;  // 不像 DHT22 位元
    if (b[i] > thr) out[i / 8] |= static_cast<uint8_t>(0x80 >> (i % 8));
  }
  return true;
}

bool checksumOk(const uint8_t b[5]) {
  return static_cast<uint8_t>(b[0] + b[1] + b[2] + b[3]) == b[4];
}

void toPhysical(const uint8_t b[5], float& t, float& h) {
  h = static_cast<float>((b[0] << 8) | b[1]) / 10.0f;
  int raw = ((b[2] & 0x7F) << 8) | b[3];
  t = static_cast<float>(raw) / 10.0f;
  if (b[2] & 0x80) t = -t;
}

}  // namespace Dht22Decode

Dht22Core::Dht22Core(IDht22Io& io, IDht22Sink& sink) : m_io(io), m_sink(sink) {}

void Dht22Core::setState(DhtState s) {
  if (s == m_state) return;
  DhtState f = m_state;
  m_state = s;
  m_sink.onStateChange(f, s);
}

void Dht22Core::fault(DhtFault c, uint32_t d) {
  m_sink.onFault(c, d);
  m_armed = false;
  m_faultTicks = 0;
  setState(DhtState::FAULT);
}

bool Dht22Core::countErr(uint8_t& ctr, DhtFault c, uint32_t d) {
  m_totalErrors++;
  if (++ctr >= m_cfg.errMax) { fault(c, d); return true; }
  return false;
}

void Dht22Core::tick(double t_s) {
  switch (m_state) {
    case DhtState::OFF:
      setState(DhtState::INIT);
      // fallthrough
    case DhtState::INIT:
      if (!m_io.configure()) { fault(DhtFault::IO_INIT, 0); return; }
      m_armed = false;
      m_good = 0;
      m_stuck = 0;
      m_lastRaw = 0xFFFFFFFF;
      m_errNoResp = m_errCrc = m_errRange = 0;
      setState(DhtState::SELFTEST);
      return;
    case DhtState::SELFTEST:
    case DhtState::READY:
      if (!m_armed) {
        if (!m_io.start()) { countErr(m_errNoResp, DhtFault::NO_RESPONSE, 0); return; }
        m_trigT = t_s;
        m_armed = true;
      } else {
        m_armed = false;
        decode();
      }
      return;
    case DhtState::FAULT:
      if (m_retries < m_cfg.maxRetries && ++m_faultTicks >= m_cfg.faultRetryTicks) {
        m_retries++;
        setState(DhtState::INIT);
      }
      return;
  }
}

void Dht22Core::decode() {
  uint16_t highs[64];
  int n = m_io.fetchHighs(highs, 64);
  if (n < 0) { countErr(m_errNoResp, DhtFault::NO_RESPONSE, 0); return; }
  m_errNoResp = 0;
  uint8_t b[5];
  if (!Dht22Decode::bitsFromHighs(highs, n, m_cfg.bitThreshold_us, b) || !Dht22Decode::checksumOk(b)) {
    countErr(m_errCrc, DhtFault::CHECKSUM, static_cast<uint32_t>(n));
    return;
  }
  m_errCrc = 0;
  float tC, hPct;
  Dht22Decode::toPhysical(b, tC, hPct);
  if (tC < -40.0f || tC > 80.0f || hPct < 0.0f || hPct > 100.0f) {
    countErr(m_errRange, DhtFault::OUT_OF_RANGE, (static_cast<uint32_t>(b[0]) << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
    return;
  }
  m_errRange = 0;
  uint32_t raw = (static_cast<uint32_t>(b[0]) << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
  if (raw == m_lastRaw) {
    if (m_cfg.stuckN > 0 && ++m_stuck >= m_cfg.stuckN) { fault(DhtFault::STUCK, raw); return; }
  } else {
    m_stuck = 0;
  }
  m_lastRaw = raw;

  HumTempOut s{m_trigT, m_seq++, m_state == DhtState::READY, tC + 273.15f, hPct / 100.0f};
  m_sink.onSample(s);
  if (m_state == DhtState::SELFTEST && ++m_good >= m_cfg.selftestN) setState(DhtState::READY);
}

}  // namespace EnvSensing
