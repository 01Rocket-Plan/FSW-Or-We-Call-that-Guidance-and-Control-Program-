#include "RfDriverSimDevice.hpp"

namespace Components {

RfDriverSimDevice::RfDriverSimDevice() = default;

uint32_t RfDriverSimDevice::rng() {
  m_rng = m_rng * 1664525u + 1013904223u;
  return m_rng;
}

bool RfDriverSimDevice::corruptByte() {
  if (m_ber <= 0.0) return false;
  const double r = static_cast<double>(rng() % 1000000u) / 1000000.0;
  return r < m_ber;
}

void RfDriverSimDevice::advance() { ++m_ticks; }

bool RfDriverSimDevice::init() {
  if (m_fault == RfSimFault::INIT_FAIL) return false;
  return true;
}

int32_t RfDriverSimDevice::writeFrame(const uint8_t* data, uint32_t len) {
  if (m_fault == RfSimFault::WRITE_FAIL) return -1;
  if (m_dropRate > 0.0) {
    const double r = static_cast<double>(rng() % 1000000u) / 1000000.0;
    if (r < m_dropRate) {
      ++m_framesDropped;
      return 0;
    }
  }
  bool corrupted = false;
  for (uint32_t i = 0; i < len; ++i) {
    if (corruptByte()) {
      corrupted = true;
      break;
    }
  }
  if (corrupted) ++m_framesCorrupted;
  (void)data;
  (void)m_latencyTicks;  // 延遲由上層 tick 節奏體現；此處僅保留參數
  ++m_framesWritten;
  return static_cast<int32_t>(len);
}

int32_t RfDriverSimDevice::readBytes(uint8_t* data, uint32_t maxLen) {
  if (m_uplinkCount == 0) return 0;
  const uint32_t n = m_uplinkCount < maxLen ? m_uplinkCount : maxLen;
  for (uint32_t i = 0; i < n; ++i) {
    data[i] = m_uplinkBuf[(m_uplinkRd + i) % kUplinkCap];
  }
  m_uplinkRd = (m_uplinkRd + n) % kUplinkCap;
  m_uplinkCount -= n;
  return static_cast<int32_t>(n);
}

bool RfDriverSimDevice::selftestLoopback() {
  if (m_fault == RfSimFault::SELFTEST_FAIL) return false;
  return true;
}

bool RfDriverSimDevice::ready() {
  if (m_fault == RfSimFault::NOT_READY) return false;
  return true;
}

void RfDriverSimDevice::pushUplink(const uint8_t* data, uint32_t len) {
  for (uint32_t i = 0; i < len; ++i) {
    if (m_uplinkCount >= kUplinkCap) break;
    const uint32_t wr = (m_uplinkRd + m_uplinkCount) % kUplinkCap;
    m_uplinkBuf[wr] = data[i];
    ++m_uplinkCount;
  }
}

}  // namespace Components
