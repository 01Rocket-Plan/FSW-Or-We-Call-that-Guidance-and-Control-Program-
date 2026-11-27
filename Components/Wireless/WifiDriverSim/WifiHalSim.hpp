// ======================================================================
// \title  WifiHalSim.hpp
// \author 無線組（Viktor）
// \brief  WiFi HAL 模擬器（對齊 BleHalSim）：主機測試與之後 SIL 用。
// ======================================================================
#ifndef WIRELESS_WIFIHALSIM_HPP
#define WIRELESS_WIFIHALSIM_HPP

#include <cstdint>
#include <deque>
#include <vector>

#include "Components/Wireless/WifiDriver/IWifiHal.hpp"

namespace Components {

class WifiHalSim final : public IWifiHal {
 public:
  WifiHalSim() = default;

  bool stackStart(const WifiRadioConfig& cfg) override;
  bool stackAlive() override;
  bool selftest() override;
  bool linkUp() override;
  bool send(const uint8_t* data, uint32_t len) override;
  int32_t pollRecv(uint8_t* data, uint32_t maxLen) override;
  int8_t rssi_dBm() override;

  //! 每拍呼叫：把本拍可送槽數恢復為 simTxSlotsPerTick
  void advance();

  void setStackStartOk(bool ok) { m_stackStartOk = ok; }
  void setStackAlive(bool alive) { m_stackAlive = alive; }
  void setSelftestOk(bool ok) { m_selftestOk = ok; }
  void setAssociated(bool assoc) { m_associated = assoc; }
  void setPeerAvailable(bool peer) { m_peerAvailable = peer; }
  void setSendSlotsPerTick(int slots) {
    m_sendSlotsPerTick = slots;
    m_sendSlots = slots;
  }
  void setLoopback(bool loop) { m_loopback = loop; }
  void setRssi(int8_t rssi) { m_rssi = rssi; }
  void injectRx(const uint8_t* data, uint32_t len);

  const WifiRadioConfig& lastConfig() const { return m_cfg; }
  uint32_t sentCount() const { return m_sentCount; }
  uint32_t lastSentLen() const { return static_cast<uint32_t>(m_lastSent.size()); }
  const uint8_t* lastSent() const { return m_lastSent.empty() ? nullptr : m_lastSent.data(); }
  bool learnedPeer() const { return m_learnedPeer; }

 private:
  WifiRadioConfig m_cfg{};
  bool m_started = false;
  bool m_stackStartOk = true;
  bool m_stackAlive = true;
  bool m_selftestOk = true;
  bool m_associated = false;
  bool m_peerAvailable = false;
  bool m_loopback = false;
  bool m_learnedPeer = false;
  int m_sendSlotsPerTick = 4;
  int m_sendSlots = 4;
  int8_t m_rssi = -50;
  uint32_t m_sentCount = 0;
  std::vector<uint8_t> m_lastSent;
  std::deque<std::vector<uint8_t>> m_rxQueue;
};

}  // namespace Components

#endif
