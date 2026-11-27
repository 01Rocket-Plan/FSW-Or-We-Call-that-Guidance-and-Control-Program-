// WifiDriverHostTest：RfDriverCore（原封不動）＋WifiLinkCore＋WifiHalSim。
// 驗證：握手、未關聯不 FAULT、UDP 一幀一包迴路、忙碌續送、斷線、溢位、
//       空 HAL／無 WiFi 型號、憑證缺失、usesLoraAirtime、參數夾取、SoftAP。
// 測試憑證是明顯假值（TEST-WIFI-SSID／test-wifi-pass-fake），不是飛行用字串。
#include "Components/Wireless/RfDriver/RfDriverCore.hpp"
#include "Components/Wireless/WifiDriver/WifiLinkCore.hpp"
#include "Components/Wireless/WifiDriverSim/WifiHalSim.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

using namespace Components;

namespace {

int g_fail = 0;
#define CHECK(c)                                                         \
  do {                                                                   \
    if (!(c)) {                                                          \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);         \
      ++g_fail;                                                          \
    }                                                                    \
  } while (0)

// TEST 假憑證：只存在於本測試檔，不進 WifiCredStore.hpp。
class FakeCredStore final : public IWifiCredStore {
 public:
  bool getSsid(char* buf, std::size_t maxLen) override {
    if (buf == nullptr || maxLen == 0) return false;
    std::strncpy(buf, "TEST-WIFI-SSID", maxLen - 1);
    buf[maxLen - 1] = '\0';
    return true;
  }
  bool getPassword(char* buf, std::size_t maxLen) override {
    if (buf == nullptr || maxLen == 0) return false;
    std::strncpy(buf, "test-wifi-pass-fake", maxLen - 1);
    buf[maxLen - 1] = '\0';
    return true;
  }
};

class Sink final : public IRfSink {
 public:
  void onStateChange(DrvState, DrvState to) override { states.push_back(to); }
  void onEvent(RfEvent ev, uint32_t) override { events.push_back(ev); }
  void onUplink(const uint8_t* d, uint32_t n) override { uplink.insert(uplink.end(), d, d + n); }
  void onComStatus(bool r) override { comReady = r; }
  bool has(RfEvent e) const {
    for (RfEvent x : events)
      if (x == e) return true;
    return false;
  }
  std::vector<DrvState> states;
  std::vector<RfEvent> events;
  std::vector<uint8_t> uplink;
  bool comReady = false;
};

class Txq final : public IRfTxQueue {
 public:
  WifiLinkCore* link = nullptr;
  uint32_t popFrame(uint8_t* buf, uint32_t maxLen) override {
    if (link != nullptr && link->txPending()) return 0;
    if (frames.empty()) return 0;
    std::vector<uint8_t> f = frames.front();
    frames.erase(frames.begin());
    if (f.size() > maxLen) return 0;
    for (size_t i = 0; i < f.size(); ++i) buf[i] = f[i];
    return static_cast<uint32_t>(f.size());
  }
  std::vector<std::vector<uint8_t>> frames;
};

std::vector<uint8_t> pattern(uint32_t n, uint8_t seed) {
  std::vector<uint8_t> v(n);
  for (uint32_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(seed + i * 7u);
  return v;
}

struct Rig {
  WifiHalSim hal;
  FakeCredStore creds;
  WifiLinkCore link;
  Sink sink;
  Txq txq;
  RfDriverCore drv;
  double t = 0.0;
  explicit Rig(const WifiRadioConfig& c = WifiRadioConfig{})
      : link(hal, creds), drv(link, sink, txq) {
    txq.link = &link;
    link.configure(c);
    drv.setTxDoneSource(&link);
    drv.configure(RfParams{});
    drv.start();
  }
  void step(int n = 1) {
    for (int i = 0; i < n; ++i) {
      hal.advance();
      link.service();
      drv.tick(t);
      t += 0.01;
    }
  }
};

void test_sta_loopback() {
  Rig r;
  r.hal.setAssociated(true);
  r.hal.setPeerAvailable(true);
  r.hal.setLoopback(true);
  r.step(5);
  CHECK(r.drv.state() == DrvState::READY);
  const std::vector<uint8_t> f = pattern(16, 3);
  r.txq.frames.push_back(f);
  r.step(3);
  CHECK(r.hal.sentCount() == 1u);
  CHECK(r.hal.lastSentLen() == f.size());
  CHECK(std::memcmp(r.hal.lastSent(), f.data(), f.size()) == 0);
  CHECK(r.drv.sentFrames() == 1u);
  CHECK(r.link.rxFrames() == 1u);
  CHECK(r.sink.uplink.size() == f.size());
  CHECK(std::memcmp(r.sink.uplink.data(), f.data(), f.size()) == 0);
}

void test_ready_without_link_never_faults() {
  Rig r;
  r.step(5);
  CHECK(r.drv.state() == DrvState::READY);
  CHECK(!r.link.linkUp());
  CHECK(r.link.ready());
  for (int i = 0; i < 10; ++i) r.txq.frames.push_back(pattern(40, 1));
  r.step(800);
  CHECK(r.drv.state() == DrvState::READY);
  CHECK(r.link.noLinkDrops() == 10u);
  CHECK(r.drv.txDroppedCount() == 10u);
  CHECK(r.hal.sentCount() == 0u);
}

void test_busy_retry_and_reject() {
  Rig r;
  r.hal.setAssociated(true);
  r.hal.setPeerAvailable(true);
  r.hal.setSendSlotsPerTick(0);
  r.step(5);
  const std::vector<uint8_t> f = pattern(8, 9);
  r.txq.frames.push_back(f);
  r.step(1);
  CHECK(r.link.txPending());
  CHECK(r.hal.sentCount() == 0u);
  CHECK(r.drv.sentFrames() == 0u);
  // 上一幀還沒送完：佇列下一幀不得被當成已送
  r.txq.frames.push_back(pattern(8, 11));
  r.step(1);
  CHECK(r.link.txPending());
  CHECK(r.txq.frames.size() == 1u);
  CHECK(r.hal.sentCount() == 0u);
  // 恢復槽數後續送第一幀（同一拍 Core 也可能接著送出佇列裡的第二幀）
  r.hal.setSendSlotsPerTick(4);
  r.step(2);
  CHECK(!r.link.txPending());
  CHECK(r.hal.sentCount() >= 1u);
  CHECK(r.drv.sentFrames() >= 1u);
  CHECK(r.txq.frames.empty());
}

void test_direct_busy_drop() {
  WifiHalSim hal;
  FakeCredStore creds;
  WifiLinkCore core(hal, creds);
  CHECK(core.init());
  hal.setAssociated(true);
  hal.setPeerAvailable(true);
  hal.setSendSlotsPerTick(0);
  core.service();
  uint8_t a[4] = {1, 2, 3, 4};
  CHECK(core.writeFrame(a, sizeof(a)) == static_cast<int32_t>(sizeof(a)));
  CHECK(core.txPending());
  uint8_t b[4] = {5, 6, 7, 8};
  CHECK(core.writeFrame(b, sizeof(b)) == 0);
  CHECK(core.busyDrops() == 1u);
}

void test_disconnect_aborts_tx() {
  Rig r;
  r.hal.setAssociated(true);
  r.hal.setPeerAvailable(true);
  r.hal.setSendSlotsPerTick(0);
  r.step(5);
  r.txq.frames.push_back(pattern(8, 2));
  r.step(1);
  CHECK(r.link.txPending());
  r.hal.setAssociated(false);
  r.step(2);
  CHECK(!r.link.txPending());
  CHECK(r.link.linkDownCount() >= 1u);
  CHECK(r.link.txAbortedOnLinkLoss() >= 1u);
  CHECK(r.drv.state() == DrvState::READY);
}

void test_overflow() {
  WifiHalSim hal;
  FakeCredStore creds;
  WifiLinkCore core(hal, creds);
  CHECK(core.init());
  hal.setAssociated(true);
  hal.setPeerAvailable(true);
  core.service();
  uint8_t big[kWifiMaxFrame + 10];
  std::memset(big, 0xAA, sizeof(big));
  hal.injectRx(big, static_cast<uint32_t>(sizeof(big)));
  core.service();
  CHECK(core.rxOverflowDrops() == 1u);
  CHECK(core.rxFrames() == 0u);
  uint8_t rx[kWifiMaxFrame];
  CHECK(core.readBytes(rx, sizeof(rx)) == 0);
}

void test_unsupported_and_empty_hal() {
  WifiHalSim hal;
  FakeCredStore creds;
  WifiLinkCore core(hal, creds);
  WifiRadioConfig c;
  c.hwModel = WifiHwKind::UNSUPPORTED;
  core.configure(c);
  CHECK(!core.init());
  CHECK(!core.ready());

  WifiHalSim hal2;
  WifiLinkCore core2(hal2, creds);
  hal2.setStackStartOk(false);
  CHECK(!core2.init());

  // 經 RfDriverCore：INIT 失敗 → FAULT
  Rig r;
  r.hal.setStackStartOk(false);
  r.step(3);
  CHECK(r.drv.state() == DrvState::FAULT);
  CHECK(r.sink.has(RfEvent::FAULT));
}

void test_missing_creds() {
  WifiHalSim hal;
  EmptyWifiCredStore creds;
  WifiLinkCore core(hal, creds);
  CHECK(!core.init());
  CHECK(!core.ready());
}

void test_airtime_txdone() {
  WifiHalSim hal;
  FakeCredStore creds;
  WifiLinkCore core(hal, creds);
  CHECK(!core.usesLoraAirtime());
  Rig r;
  r.hal.setAssociated(true);
  r.hal.setPeerAvailable(true);
  r.step(5);
  r.txq.frames.push_back(pattern(12, 4));
  r.step(2);
  CHECK(r.drv.sentFrames() == 1u);
  CHECK(r.drv.lastTxAirtimeUs() == 0u);
}

void test_clamp() {
  CHECK(WifiLinkCore::clampChannel(0, WifiHalMode::STA) == 0);
  CHECK(WifiLinkCore::clampChannel(20, WifiHalMode::STA) == 13);
  CHECK(WifiLinkCore::clampChannel(0, WifiHalMode::SOFT_AP) == 1);
  CHECK(WifiLinkCore::clampChannel(20, WifiHalMode::SOFT_AP) == 13);
  CHECK(WifiLinkCore::clampChannel(6, WifiHalMode::SOFT_AP) == 6);
  CHECK(WifiLinkCore::clampTxPower(0) == kWifiTxPowerMinEsp32_dBm);
  CHECK(WifiLinkCore::clampTxPower(30) == kWifiTxPowerMaxEsp32_dBm);
  CHECK(WifiLinkCore::clampTxPower(15) == 15);

  WifiRadioConfig c;
  c.channel = 99;
  c.txPower_dBm = 1;
  c.mode = WifiHalMode::SOFT_AP;
  c.hwModel = static_cast<WifiHwKind>(9);
  WifiRadioConfig o = clampWifiConfig(c);
  CHECK(o.hwModel == WifiHwKind::UNSUPPORTED);
  CHECK(o.channel == 13);
  CHECK(o.txPower_dBm == kWifiTxPowerMinEsp32_dBm);
}

void test_softap_loopback() {
  WifiRadioConfig c;
  c.mode = WifiHalMode::SOFT_AP;
  c.channel = 6;
  Rig r(c);
  CHECK(r.link.channel() == 6);
  r.hal.setAssociated(true);
  r.hal.setPeerAvailable(true);
  r.hal.setLoopback(true);
  r.step(5);
  const std::vector<uint8_t> f = {1, 2, 3, 4};
  r.txq.frames.push_back(f);
  r.step(3);
  CHECK(r.hal.sentCount() == 1u);
  CHECK(r.sink.uplink == f);
  CHECK(r.drv.sentFrames() == 1u);
}

void test_learn_peer_from_rx() {
  WifiRadioConfig c;
  c.learnPeerFromRx = true;
  WifiHalSim hal;
  FakeCredStore creds;
  WifiLinkCore core(hal, creds);
  core.configure(c);
  CHECK(core.init());
  CHECK(hal.lastConfig().learnPeerFromRx);
  hal.setAssociated(true);
  hal.setPeerAvailable(false);
  core.service();
  CHECK(!core.linkUp());
  uint8_t f[4] = {9, 8, 7, 6};
  hal.injectRx(f, sizeof(f));
  core.service();
  CHECK(hal.learnedPeer());
  CHECK(core.linkUp());
  CHECK(core.rxFrames() == 1u);
}

}  // namespace

int WifiDriverLegacyMain() {
  g_fail = 0;
  struct T {
    const char* name;
    void (*fn)();
  } tests[] = {
      {"sta_loopback", test_sta_loopback},
      {"ready_without_link_never_faults", test_ready_without_link_never_faults},
      {"busy_retry_and_reject", test_busy_retry_and_reject},
      {"direct_busy_drop", test_direct_busy_drop},
      {"disconnect_aborts_tx", test_disconnect_aborts_tx},
      {"overflow", test_overflow},
      {"unsupported_and_empty_hal", test_unsupported_and_empty_hal},
      {"missing_creds", test_missing_creds},
      {"airtime_txdone", test_airtime_txdone},
      {"clamp", test_clamp},
      {"softap_loopback", test_softap_loopback},
      {"learn_peer_from_rx", test_learn_peer_from_rx},
  };
  for (const T& t : tests) {
    const int before = g_fail;
    t.fn();
    std::printf("[%s] %s\n", (g_fail == before) ? "PASS" : "FAIL", t.name);
  }
  std::printf("WifiDriver host: %d failure(s)\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}

#ifdef HOST_TEST_MAIN
int main() { return WifiDriverLegacyMain(); }
#else
#include <gtest/gtest.h>
TEST(Wireless, WifiDriverHostSuite) { ASSERT_EQ(0, WifiDriverLegacyMain()); }
#endif
