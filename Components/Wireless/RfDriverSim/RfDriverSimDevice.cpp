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

void RfDriverSimDevice::configureFrom(const RfParams& p) {
  m_modem = loraConfigFromRfParams(p);
  setTickPeriodUs(p.tickPeriod_us);
}

uint32_t RfDriverSimDevice::airtimeTicks(uint32_t len) const {
  return loraAirtimeTicks(m_modem, len, m_tickPeriod_us);  // ceil(ToA / tickPeriod)，整數運算
}

int32_t RfDriverSimDevice::writeFrame(const uint8_t* data, uint32_t len) {
  if (m_fault == RfSimFault::WRITE_FAIL) return -1;
  if (m_airtimeEnabled) {
    if (txBusy()) {
      // 上一包還在空中：SX1278 一次只能發一包（FIFO 共用），這一幀不收
      ++m_framesRejectedBusy;
      return 0;
    }
    const uint32_t n = airtimeTicks(len);
    if (n == 0u) {
      // PL 超出 1..255 或數據機設定無效：真機發不出去
      ++m_framesRejectedLength;
      return 0;
    }
    // 無線電佔線從這一包送出開始算；sim 掉包＝空中遺失，仍然佔用空中時間
    m_busyUntilTick = (n > 0xFFFFFFFFu - m_ticks) ? 0xFFFFFFFFu : m_ticks + n;  // 飽和加法
    const uint64_t toa = loraTimeOnAirUs(m_modem, len);
    m_lastAirtime_us = (toa > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(toa);
  }
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
  ++m_framesWritten;
  return static_cast<int32_t>(len);
}

int32_t RfDriverSimDevice::readBytes(uint8_t* data, uint32_t maxLen) {
  // 丟掉已經讀完的段；找出目前可讀到哪個累計位元組（latencyTicks）
  while (m_uplinkSegCount > 0 &&
         (m_uplinkSeg[m_uplinkSegRd].endTotal - m_uplinkReadTotal) == 0u) {
    m_uplinkSegRd = (m_uplinkSegRd + 1u) % kUplinkSegs;
    --m_uplinkSegCount;
  }
  uint32_t readable = 0;  // 從 m_uplinkReadTotal 起算可讀的位元組數
  for (uint32_t k = 0; k < m_uplinkSegCount; ++k) {
    const UplinkSeg& seg = m_uplinkSeg[(m_uplinkSegRd + k) % kUplinkSegs];
    if (seg.readyTick > m_ticks) break;
    readable = seg.endTotal - m_uplinkReadTotal;  // 無號差：累計值繞回也正確
  }
  if (readable > m_uplinkCount) readable = m_uplinkCount;
  if (readable == 0) return 0;
  const uint32_t n = readable < maxLen ? readable : maxLen;
  for (uint32_t i = 0; i < n; ++i) {
    data[i] = m_uplinkBuf[(m_uplinkRd + i) % kUplinkCap];
  }
  m_uplinkRd = (m_uplinkRd + n) % kUplinkCap;
  m_uplinkCount -= n;
  m_uplinkReadTotal += n;
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
  uint32_t accepted = 0;
  for (uint32_t i = 0; i < len; ++i) {
    if (m_uplinkCount >= kUplinkCap) break;
    const uint32_t wr = (m_uplinkRd + m_uplinkCount) % kUplinkCap;
    m_uplinkBuf[wr] = data[i];
    ++m_uplinkCount;
    ++accepted;
  }
  if (accepted == 0) return;
  m_uplinkPushedTotal += accepted;
  // 飽和加法：避免 m_ticks + latency 溢位
  const uint32_t ready = (m_latencyTicks > 0xFFFFFFFFu - m_ticks) ? 0xFFFFFFFFu : m_ticks + m_latencyTicks;
  if (m_uplinkSegCount == kUplinkSegs) {
    // 段數滿：併進最後一段（可讀時間取較晚者，不會讓位元組提早送達）
    UplinkSeg& last = m_uplinkSeg[(m_uplinkSegRd + m_uplinkSegCount - 1u) % kUplinkSegs];
    last.endTotal = m_uplinkPushedTotal;
    if (ready > last.readyTick) last.readyTick = ready;
    return;
  }
  UplinkSeg& seg = m_uplinkSeg[(m_uplinkSegRd + m_uplinkSegCount) % kUplinkSegs];
  seg.endTotal = m_uplinkPushedTotal;
  seg.readyTick = ready;
  ++m_uplinkSegCount;
}

}  // namespace Components
