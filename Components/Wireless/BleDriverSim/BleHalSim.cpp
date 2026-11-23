// BleHalSim：STM32WB55 BLE 的 IBleHal 模擬。說明見 BleHalSim.hpp。
#include "Components/Wireless/BleDriverSim/BleHalSim.hpp"

namespace Components {

BleHalSim::BleHalSim() = default;

void BleHalSim::setLinkUp(bool up) {
  if (up && !m_link) {
    m_linkTicks = 0u;
    m_peerTxSeq = 0u;
    m_peerInFrame = false;
    m_peerAsmLen = 0u;
  }
  if (!up) {
    m_downTicks = 0u;
    m_writeCount = 0u;  // 斷線：中心端還沒送達的寫入都沒了
    m_writeRd = 0u;
    m_peerInFrame = false;
    m_peerAsmLen = 0u;
  }
  m_link = up;
}

void BleHalSim::setAutoLink(uint32_t dropEveryTicks, uint32_t reconnectAfterTicks) {
  m_dropEvery = dropEveryTicks;
  m_reconnectAfter = reconnectAfterTicks;
}

void BleHalSim::advance() {
  m_txUsedThisTick = 0u;
  if (!m_started) return;
  if (m_link) {
    ++m_linkTicks;
    if (m_dropEvery > 0u && m_linkTicks >= m_dropEvery) setLinkUp(false);
  } else {
    ++m_downTicks;
    if (m_reconnectAfter > 0u && m_downTicks >= m_reconnectAfter) setLinkUp(true);
  }
}

bool BleHalSim::stackStart(const BleRadioConfig& cfg) {
  ++m_startCalls;
  m_lastCfg = cfg;
  m_started = (m_fault != BleSimFault::START_FAIL);
  return m_started;
}

bool BleHalSim::stackAlive() { return m_started && m_fault != BleSimFault::STACK_DEAD; }

bool BleHalSim::selftest() { return stackAlive() && m_fault != BleSimFault::SELFTEST_FAIL; }

bool BleHalSim::linkUp() { return stackAlive() && m_link; }

uint16_t BleHalSim::attMtu() { return linkUp() ? m_mtu : kBleAttMtuMin; }

bool BleHalSim::notify(const uint8_t* data, uint16_t len) {
  if (!linkUp() || data == nullptr || len == 0u) return false;
  if (len > static_cast<uint16_t>(attMtu() - kBleAttOverhead)) {
    ++m_peerFragErrors;  // 超過 MTU：真機會被堆疊拒絕
    return false;
  }
  if (m_txUsedThisTick >= m_txSlotsPerTick) {
    ++m_notifyRejectedFull;
    return false;
  }
  ++m_txUsedThisTick;
  ++m_peerFragments;
  if (len > m_maxNotifyLen) m_maxNotifyLen = len;

  // 中心端重組（與 BleLinkCore 同一套片頭規則）
  const uint8_t hdr = data[0];
  const uint8_t seq = static_cast<uint8_t>(hdr & kBleFragSeqMask);
  if ((hdr & kBleFragFirst) != 0u) {
    if (m_peerInFrame) ++m_peerFragErrors;
    m_peerInFrame = true;
    m_peerAsmLen = 0u;
  } else if (!m_peerInFrame || seq != m_peerExpect) {
    ++m_peerFragErrors;
    m_peerInFrame = false;
    m_peerExpect = static_cast<uint8_t>((seq + 1u) & kBleFragSeqMask);
    return true;
  }
  m_peerExpect = static_cast<uint8_t>((seq + 1u) & kBleFragSeqMask);
  const uint32_t plen = static_cast<uint32_t>(len) - 1u;
  if (m_peerAsmLen + plen > kBleMaxFrame) {
    ++m_peerFragErrors;
    m_peerInFrame = false;
    return true;
  }
  for (uint32_t i = 0; i < plen; ++i) m_peerAsm[m_peerAsmLen + i] = data[1 + i];
  m_peerAsmLen += plen;
  if ((hdr & kBleFragLast) != 0u) {
    for (uint32_t i = 0; i < m_peerAsmLen; ++i) m_lastPeerFrame[i] = m_peerAsm[i];
    m_lastPeerFrameLen = m_peerAsmLen;
    ++m_peerFrames;
    m_peerInFrame = false;
  }
  return true;
}

bool BleHalSim::peerWriteFragment(const uint8_t* data, uint16_t len) {
  if (!linkUp() || data == nullptr || len == 0u || len > kBleAttMtuMax) return false;
  if (m_writeCount >= kWriteSlots) return false;
  const uint32_t slot = (m_writeRd + m_writeCount) % kWriteSlots;
  for (uint16_t i = 0; i < len; ++i) m_writeBuf[slot][i] = data[i];
  m_writeLen[slot] = len;
  ++m_writeCount;
  return true;
}

bool BleHalSim::peerWriteFrame(const uint8_t* data, uint32_t len, uint16_t fragPayload) {
  if (data == nullptr || len == 0u || fragPayload == 0u) return false;
  uint8_t frag[kBleAttMtuMax];
  uint32_t off = 0u;
  while (off < len) {
    const uint32_t remain = len - off;
    const uint32_t n = (remain < fragPayload) ? remain : fragPayload;
    uint8_t hdr = static_cast<uint8_t>(m_peerTxSeq & kBleFragSeqMask);
    if (off == 0u) hdr = static_cast<uint8_t>(hdr | kBleFragFirst);
    if (off + n == len) hdr = static_cast<uint8_t>(hdr | kBleFragLast);
    frag[0] = hdr;
    for (uint32_t i = 0; i < n; ++i) frag[1 + i] = data[off + i];
    if (!peerWriteFragment(frag, static_cast<uint16_t>(n + 1u))) return false;
    m_peerTxSeq = static_cast<uint8_t>((m_peerTxSeq + 1u) & kBleFragSeqMask);
    off += n;
  }
  return true;
}

int32_t BleHalSim::pollWrite(uint8_t* data, uint16_t maxLen) {
  if (!linkUp() || m_writeCount == 0u || data == nullptr) return 0;
  const uint16_t len = m_writeLen[m_writeRd];
  if (len > maxLen) return 0;
  for (uint16_t i = 0; i < len; ++i) data[i] = m_writeBuf[m_writeRd][i];
  m_writeRd = (m_writeRd + 1u) % kWriteSlots;
  --m_writeCount;
  return static_cast<int32_t>(len);
}

}  // namespace Components
