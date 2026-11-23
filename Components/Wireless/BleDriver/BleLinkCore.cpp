// BleLinkCore：BLE GATT 切片／重組＋掉線處理。行為說明見 BleLinkCore.hpp 檔頭。
#include "Components/Wireless/BleDriver/BleLinkCore.hpp"

namespace Components {

namespace {
// AN5270 aci_hal_set_tx_power_level：PA_Level 0x00＝−40 dBm … 0x1F＝+6 dBm；DS11929：最大 +6 dBm
constexpr int8_t kTxPowerMin_dBm = -40;
constexpr int8_t kTxPowerMax_dBm = 6;
}  // namespace

BleRadioConfig clampBleConfig(const BleRadioConfig& in) {
  BleRadioConfig c = in;
  if (c.txPower_dBm < kTxPowerMin_dBm) c.txPower_dBm = kTxPowerMin_dBm;
  if (c.txPower_dBm > kTxPowerMax_dBm) c.txPower_dBm = kTxPowerMax_dBm;
  if (c.attMtu < kBleAttMtuMin) c.attMtu = kBleAttMtuMin;
  if (c.attMtu > kBleAttMtuMax) c.attMtu = kBleAttMtuMax;
  if (c.phy != BlePhyMode::LE_1M && c.phy != BlePhyMode::LE_2M) c.phy = BlePhyMode::LE_1M;
  return c;
}

BleLinkCore::BleLinkCore(IBleHal& hal) : m_hal(hal) {}

uint16_t BleLinkCore::fragPayload() const {
  uint16_t mtu = m_hal.attMtu();
  if (mtu < kBleAttMtuMin) mtu = kBleAttMtuMin;
  if (mtu > m_cfg.attMtu) mtu = m_cfg.attMtu;  // 不超過我們自己要求的上限
  return static_cast<uint16_t>(mtu - kBleAttOverhead - 1u);  // 再扣 1 B 片頭
}

bool BleLinkCore::init() {
  // 重新 INIT（RfDriverCore FAULT 後由外殼 start() 再來）也要清乾淨
  m_txLen = 0;
  m_txOff = 0;
  m_rxInFrame = false;
  m_rxAsmLen = 0;
  m_rxFifoRd = 0;
  m_rxFifoCount = 0;
  m_linkUp = false;
  m_started = m_hal.stackStart(m_cfg);
  return m_started;
}

bool BleLinkCore::selftestLoopback() { return m_started && m_hal.stackAlive() && m_hal.selftest(); }

bool BleLinkCore::ready() {
  // 「有沒有連線」不算 ready 條件（見檔頭）；只看 CPU2 堆疊活著
  return m_started && m_hal.stackAlive();
}

void BleLinkCore::onLinkChange(bool up) {
  if (up == m_linkUp) return;
  m_linkUp = up;
  if (up) {
    ++m_linkUpCount;
    m_txSeq = 0;
    m_rxExpectSeq = 0;
    return;
  }
  ++m_linkDownCount;
  if (m_txLen > 0) {
    ++m_txAbortedOnLinkLoss;  // 送一半的幀不跨連線續送
    m_txLen = 0;
    m_txOff = 0;
  }
  dropPartialRx();
}

void BleLinkCore::service() {
  if (!m_started) return;
  onLinkChange(m_hal.linkUp());
  if (!m_linkUp) return;
  flushTx();
  pollRx();
}

int32_t BleLinkCore::writeFrame(const uint8_t* data, uint32_t len) {
  if (data == nullptr || len == 0u || len > kBleMaxFrame) return 0;
  if (!m_started || !m_linkUp) {
    ++m_noLinkDrops;
    return 0;
  }
  if (m_txLen > 0) {
    ++m_busyDrops;  // 上一幀還沒送完：不排隊（排隊屬 ComQueue），直接丟
    return 0;
  }
  for (uint32_t i = 0; i < len; ++i) m_txBuf[i] = data[i];
  m_txLen = len;
  m_txOff = 0;
  flushTx();
  return static_cast<int32_t>(len);
}

void BleLinkCore::flushTx() {
  if (m_txLen == 0) return;
  const uint16_t per = fragPayload();
  uint8_t frag[kBleAttMtuMax];
  while (m_txOff < m_txLen) {
    const uint32_t remain = m_txLen - m_txOff;
    const uint32_t n = (remain < per) ? remain : per;
    uint8_t hdr = static_cast<uint8_t>(m_txSeq & kBleFragSeqMask);
    if (m_txOff == 0) hdr = static_cast<uint8_t>(hdr | kBleFragFirst);
    if (m_txOff + n == m_txLen) hdr = static_cast<uint8_t>(hdr | kBleFragLast);
    frag[0] = hdr;
    for (uint32_t i = 0; i < n; ++i) frag[1 + i] = m_txBuf[m_txOff + i];
    if (!m_hal.notify(frag, static_cast<uint16_t>(n + 1u))) {
      return;  // CPU2 緩衝滿：下一拍 service() 再送
    }
    ++m_fragmentsSent;
    m_txSeq = static_cast<uint8_t>((m_txSeq + 1u) & kBleFragSeqMask);
    m_txOff += n;
  }
  ++m_framesFullySent;
  m_txLen = 0;
  m_txOff = 0;
}

void BleLinkCore::dropPartialRx() {
  if (m_rxInFrame) ++m_rxFragErrors;
  m_rxInFrame = false;
  m_rxAsmLen = 0;
}

void BleLinkCore::onRxFragment(const uint8_t* d, uint32_t n) {
  if (n < 1u) {
    ++m_rxFragErrors;
    return;
  }
  const uint8_t hdr = d[0];
  const uint8_t seq = static_cast<uint8_t>(hdr & kBleFragSeqMask);
  const uint8_t* payload = d + 1;
  const uint32_t plen = n - 1u;

  if ((hdr & kBleFragFirst) != 0u) {
    if (m_rxInFrame) dropPartialRx();  // 上一幀沒收完就來新幀 → 舊的丟
    m_rxInFrame = true;
    m_rxAsmLen = 0;
  } else if (!m_rxInFrame || seq != m_rxExpectSeq) {
    // 沒有開頭的中段片，或序號跳號 → 整幀作廢，等下一個 FIRST
    ++m_rxFragErrors;
    m_rxInFrame = false;
    m_rxAsmLen = 0;
    m_rxExpectSeq = static_cast<uint8_t>((seq + 1u) & kBleFragSeqMask);
    return;
  }
  m_rxExpectSeq = static_cast<uint8_t>((seq + 1u) & kBleFragSeqMask);

  if (m_rxAsmLen + plen > kBleMaxFrame) {
    dropPartialRx();
    return;
  }
  for (uint32_t i = 0; i < plen; ++i) m_rxAsm[m_rxAsmLen + i] = payload[i];
  m_rxAsmLen += plen;

  if ((hdr & kBleFragLast) != 0u) {
    // 整幀完成 → 放進位元組 FIFO；放不下就整幀丟（不放半幀，免得 Framer 收到殘缺幀）
    if (m_rxAsmLen > kRxFifoCap - m_rxFifoCount) {
      ++m_rxOverflowDrops;
    } else {
      for (uint32_t i = 0; i < m_rxAsmLen; ++i) {
        m_rxFifo[(m_rxFifoRd + m_rxFifoCount + i) % kRxFifoCap] = m_rxAsm[i];
      }
      m_rxFifoCount += m_rxAsmLen;
      ++m_rxFrames;
    }
    m_rxInFrame = false;
    m_rxAsmLen = 0;
  }
}

void BleLinkCore::pollRx() {
  uint8_t frag[kBleAttMtuMax];
  // 每拍最多處理固定片數，避免單拍時間失控
  for (int k = 0; k < 16; ++k) {
    const int32_t n = m_hal.pollWrite(frag, static_cast<uint16_t>(sizeof(frag)));
    if (n <= 0) return;
    onRxFragment(frag, static_cast<uint32_t>(n));
  }
}

int32_t BleLinkCore::readBytes(uint8_t* data, uint32_t maxLen) {
  if (data == nullptr || maxLen == 0u || m_rxFifoCount == 0u) return 0;
  const uint32_t n = (m_rxFifoCount < maxLen) ? m_rxFifoCount : maxLen;
  for (uint32_t i = 0; i < n; ++i) data[i] = m_rxFifo[(m_rxFifoRd + i) % kRxFifoCap];
  m_rxFifoRd = (m_rxFifoRd + n) % kRxFifoCap;
  m_rxFifoCount -= n;
  return static_cast<int32_t>(n);
}

}  // namespace Components
