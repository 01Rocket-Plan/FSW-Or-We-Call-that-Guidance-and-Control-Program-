// WifiLinkCore：UDP 一幀一包＋掉線處理。行為說明見 WifiLinkCore.hpp 檔頭。
#include "Components/Wireless/WifiDriver/WifiLinkCore.hpp"

#include <cstring>

namespace Components {

uint8_t WifiLinkCore::clampChannel(uint8_t ch, WifiHalMode mode) {
  if (mode == WifiHalMode::SOFT_AP) {
    if (ch < 1u) return 1u;
    if (ch > 13u) return 13u;
    return ch;
  }
  // STA：0 表示交給堆疊／AP；非 0 時夾到 1–13
  if (ch == 0u) return 0u;
  if (ch > 13u) return 13u;
  return ch;
}

int8_t WifiLinkCore::clampTxPower(int8_t dBm) {
  if (dBm < kWifiTxPowerMinEsp32_dBm) return kWifiTxPowerMinEsp32_dBm;
  if (dBm > kWifiTxPowerMaxEsp32_dBm) return kWifiTxPowerMaxEsp32_dBm;
  return dBm;
}

WifiRadioConfig clampWifiConfig(const WifiRadioConfig& in) {
  WifiRadioConfig c = in;
  if (c.hwModel != WifiHwKind::ESP32_INTERNAL_WIFI && c.hwModel != WifiHwKind::UNSUPPORTED) {
    c.hwModel = WifiHwKind::UNSUPPORTED;
  }
  if (c.mode != WifiHalMode::STA && c.mode != WifiHalMode::SOFT_AP) {
    c.mode = WifiHalMode::STA;
  }
  c.channel = WifiLinkCore::clampChannel(c.channel, c.mode);
  c.txPower_dBm = WifiLinkCore::clampTxPower(c.txPower_dBm);
  return c;
}

WifiLinkCore::WifiLinkCore(IWifiHal& hal, IWifiCredStore& creds) : m_hal(hal), m_creds(creds) {}

bool WifiLinkCore::fillCreds() {
  char ssid[kWifiSsidMax] = {0};
  char pass[kWifiPassMax] = {0};
  if (!m_creds.getSsid(ssid, sizeof(ssid)) || ssid[0] == '\0') return false;
  if (!m_creds.getPassword(pass, sizeof(pass)) || pass[0] == '\0') return false;
  std::strncpy(m_cfg.ssid, ssid, sizeof(m_cfg.ssid) - 1);
  m_cfg.ssid[sizeof(m_cfg.ssid) - 1] = '\0';
  std::strncpy(m_cfg.password, pass, sizeof(m_cfg.password) - 1);
  m_cfg.password[sizeof(m_cfg.password) - 1] = '\0';
  return true;
}

bool WifiLinkCore::init() {
  m_txLen = 0;
  m_txDoneFlag = false;
  m_txAbortedFlag = false;
  m_rxFifoRd = 0;
  m_rxFifoCount = 0;
  m_linkUp = false;
  m_started = false;
  m_cfg = clampWifiConfig(m_cfg);
  // 備用 MCU（或明確標 UNSUPPORTED）沒有 WiFi：不啟動堆疊
  if (m_cfg.hwModel != WifiHwKind::ESP32_INTERNAL_WIFI) {
    return false;
  }
  if (!fillCreds()) {
    return false;
  }
  m_started = m_hal.stackStart(m_cfg);
  return m_started;
}

bool WifiLinkCore::selftestLoopback() { return m_started && m_hal.stackAlive() && m_hal.selftest(); }

bool WifiLinkCore::ready() { return m_started && m_hal.stackAlive(); }

void WifiLinkCore::onLinkChange(bool up) {
  if (up == m_linkUp) return;
  m_linkUp = up;
  if (up) {
    ++m_linkUpCount;
    return;
  }
  ++m_linkDownCount;
  if (m_txLen > 0) {
    ++m_txAbortedOnLinkLoss;
    m_txAbortedFlag = true;
    m_txLen = 0;
  }
  // 不把斷線前尚未讀走的半段上行交給上層：清 FIFO
  m_rxFifoRd = 0;
  m_rxFifoCount = 0;
}

void WifiLinkCore::service() {
  if (!m_started) return;
  onLinkChange(m_hal.linkUp());
  // 尚未關聯時仍要輪詢：learnPeerFromRx 可能在此學會對端並把鏈路拉起來。
  // 學會之後這一包照常入佇列；若仍未關聯則丟棄、不計 rxFrames（不跨關聯拼接）。
  if (!m_linkUp) {
    uint8_t buf[kWifiMaxFrame];
    for (int k = 0; k < 16; ++k) {
      const int32_t raw = m_hal.pollRecv(buf, static_cast<uint32_t>(sizeof(buf)));
      if (raw <= 0) break;
      onLinkChange(m_hal.linkUp());
      if (!m_linkUp) continue;
      const uint32_t n = static_cast<uint32_t>(raw);
      if (n > kWifiMaxFrame || !enqueueRx(buf, n)) {
        ++m_rxOverflowDrops;
      } else {
        ++m_rxFrames;
      }
    }
    if (!m_linkUp) return;
  }
  flushTx();
  pollRx();
}

int32_t WifiLinkCore::writeFrame(const uint8_t* data, uint32_t len) {
  if (m_started) onLinkChange(m_hal.linkUp());
  if (data == nullptr || len == 0u || len > kWifiMaxFrame) return 0;
  if (!m_started || !m_linkUp) {
    ++m_noLinkDrops;
    return 0;
  }
  if (m_txLen > 0) {
    ++m_busyDrops;
    return 0;
  }
  for (uint32_t i = 0; i < len; ++i) m_txBuf[i] = data[i];
  m_txDoneFlag = false;
  m_txAbortedFlag = false;
  m_txLen = len;
  flushTx();
  return static_cast<int32_t>(len);
}

void WifiLinkCore::flushTx() {
  if (m_txLen == 0) return;
  if (!m_hal.send(m_txBuf, m_txLen)) {
    return;  // 忙：下一拍 service() 再試
  }
  ++m_framesFullySent;
  m_txDoneFlag = true;
  m_txLen = 0;
}

bool WifiLinkCore::takeTxDone() {
  const bool f = m_txDoneFlag;
  m_txDoneFlag = false;
  return f;
}

void WifiLinkCore::abortTx() {
  if (m_txLen > 0) {
    ++m_txAbortedOnTimeout;
    m_txLen = 0;
  }
  m_txDoneFlag = false;
  m_txAbortedFlag = false;
}

bool WifiLinkCore::takeTxAborted() {
  const bool f = m_txAbortedFlag;
  m_txAbortedFlag = false;
  return f;
}

bool WifiLinkCore::enqueueRx(const uint8_t* data, uint32_t len) {
  if (len > kRxFifoCap - m_rxFifoCount) return false;
  for (uint32_t i = 0; i < len; ++i) {
    m_rxFifo[(m_rxFifoRd + m_rxFifoCount + i) % kRxFifoCap] = data[i];
  }
  m_rxFifoCount += len;
  return true;
}

void WifiLinkCore::pollRx() {
  uint8_t buf[kWifiMaxFrame];
  for (int k = 0; k < 16; ++k) {
    const int32_t raw = m_hal.pollRecv(buf, static_cast<uint32_t>(sizeof(buf)));
    if (raw <= 0) return;
    const uint32_t n = static_cast<uint32_t>(raw);
    if (n > kWifiMaxFrame) {
      ++m_rxOverflowDrops;
      continue;
    }
    if (!enqueueRx(buf, n)) {
      ++m_rxOverflowDrops;
      continue;
    }
    ++m_rxFrames;
  }
}

int32_t WifiLinkCore::readBytes(uint8_t* data, uint32_t maxLen) {
  if (data == nullptr || maxLen == 0u || m_rxFifoCount == 0u) return 0;
  const uint32_t n = (m_rxFifoCount < maxLen) ? m_rxFifoCount : maxLen;
  for (uint32_t i = 0; i < n; ++i) data[i] = m_rxFifo[(m_rxFifoRd + i) % kRxFifoCap];
  m_rxFifoRd = (m_rxFifoRd + n) % kRxFifoCap;
  m_rxFifoCount -= n;
  return static_cast<int32_t>(n);
}

}  // namespace Components
