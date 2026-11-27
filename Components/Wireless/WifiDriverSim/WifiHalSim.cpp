// WifiHalSim：IWifiHal 的主機／測試注入實作。
#include "Components/Wireless/WifiDriverSim/WifiHalSim.hpp"

#include <cstring>

namespace Components {

bool WifiHalSim::stackStart(const WifiRadioConfig& cfg) {
  m_cfg = cfg;
  m_learnedPeer = false;
  m_rxQueue.clear();
  m_lastSent.clear();
  m_sentCount = 0;
  m_sendSlots = m_sendSlotsPerTick;
  if (!m_stackStartOk) {
    m_started = false;
    return false;
  }
  m_started = true;
  m_stackAlive = true;
  return true;
}

bool WifiHalSim::stackAlive() { return m_started && m_stackAlive; }

bool WifiHalSim::selftest() { return m_started && m_selftestOk; }

bool WifiHalSim::linkUp() { return m_started && m_associated && m_peerAvailable; }

void WifiHalSim::advance() { m_sendSlots = m_sendSlotsPerTick; }

bool WifiHalSim::send(const uint8_t* data, uint32_t len) {
  if (!linkUp() || data == nullptr || len == 0u) return false;
  if (m_sendSlots <= 0) return false;
  --m_sendSlots;
  m_lastSent.assign(data, data + len);
  ++m_sentCount;
  if (m_loopback) {
    m_rxQueue.emplace_back(data, data + len);
  }
  return true;
}

int32_t WifiHalSim::pollRecv(uint8_t* data, uint32_t maxLen) {
  if (m_rxQueue.empty()) return 0;
  const std::vector<uint8_t> pkt = m_rxQueue.front();
  m_rxQueue.pop_front();
  if (m_cfg.learnPeerFromRx) {
    m_peerAvailable = true;
    m_learnedPeer = true;
  }
  if (pkt.size() > maxLen) {
    // 消耗該包、不越界寫入，回傳實際長度讓上層計溢位
    return static_cast<int32_t>(pkt.size());
  }
  if (data != nullptr && !pkt.empty()) {
    std::memcpy(data, pkt.data(), pkt.size());
  }
  return static_cast<int32_t>(pkt.size());
}

int8_t WifiHalSim::rssi_dBm() { return m_rssi; }

void WifiHalSim::injectRx(const uint8_t* data, uint32_t len) {
  if (data == nullptr || len == 0u) return;
  m_rxQueue.emplace_back(data, data + len);
}

}  // namespace Components
