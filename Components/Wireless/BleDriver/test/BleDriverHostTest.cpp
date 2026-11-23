// BleDriverHostTest：RfDriverCore（原封不動）＋BleLinkCore＋BleHalSim。
// 驗證：握手、沒連線不 FAULT、切片／重組（MTU 247 與 23）、TX 緩衝滿、斷線中途、上行重組、堆疊故障。
#include "Components/Wireless/BleDriver/BleLinkCore.hpp"
#include "Components/Wireless/BleDriverSim/BleHalSim.hpp"
#include "Components/Wireless/Framer/FramerCore.hpp"
#include "Components/Wireless/RfDriver/RfDriverCore.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
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

class Sink : public IRfSink {
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

class Txq : public IRfTxQueue {
 public:
  uint32_t popFrame(uint8_t* buf, uint32_t maxLen) override {
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
  BleHalSim hal;
  BleLinkCore link{hal};
  Sink sink;
  Txq txq;
  RfDriverCore drv{link, sink, txq};
  double t = 0.0;
  explicit Rig(const BleRadioConfig& c = BleRadioConfig{}) {
    link.configure(c);
    drv.configure(RfParams{});
    drv.start();
  }
  // 外殼每拍的順序：sim 前進 → link.service() → drv.tick()
  void step(int n = 1) {
    for (int i = 0; i < n; ++i) {
      hal.advance();
      link.service();
      drv.tick(t);
      t += 0.01;  // rg100Hz
    }
  }
};

bool equalLast(const BleHalSim& hal, const std::vector<uint8_t>& f) {
  if (hal.lastPeerFrameLen() != f.size()) return false;
  for (size_t i = 0; i < f.size(); ++i)
    if (hal.lastPeerFrame()[i] != f[i]) return false;
  return true;
}

void test_config_clamp() {
  BleRadioConfig c;
  c.txPower_dBm = 20;
  c.attMtu = 600;
  BleRadioConfig o = clampBleConfig(c);
  CHECK(o.txPower_dBm == 6);
  CHECK(o.attMtu == 247);
  c.txPower_dBm = -100;
  c.attMtu = 5;
  o = clampBleConfig(c);
  CHECK(o.txPower_dBm == -40);
  CHECK(o.attMtu == 23);
  Rig r(c);
  r.step(3);
  CHECK(r.hal.startCalls() == 1u);
  CHECK(r.hal.lastConfig().txPower_dBm == -40);  // HAL 拿到的是夾過的值
}

void test_ready_without_link_never_faults() {
  Rig r;
  r.step(5);
  CHECK(r.drv.state() == DrvState::READY);
  CHECK(!r.link.linkUp());
  for (int i = 0; i < 10; ++i) r.txq.frames.push_back(pattern(40, 1));
  r.step(800);  // 8 s，大於 readyDeadline_s（5 s）
  CHECK(r.drv.state() == DrvState::READY);  // 沒人連不是故障
  CHECK(r.link.noLinkDrops() == 10u);
  CHECK(r.drv.txDroppedCount() == 10u);
  CHECK(r.hal.peerFrames() == 0u);
}

void test_frag_mtu247() {
  Rig r;
  r.hal.setLinkUp(true);
  r.step(5);
  CHECK(r.drv.state() == DrvState::READY);
  CHECK(r.link.fragPayload() == 243u);  // 247 − 3（ATT）− 1（片頭）
  const std::vector<uint8_t> f = pattern(255, 3);
  r.txq.frames.push_back(f);
  r.step(3);
  CHECK(r.hal.peerFrames() == 1u);
  CHECK(r.hal.peerFragments() == 2u);
  CHECK(r.hal.maxNotifyLen() <= 244u);
  CHECK(equalLast(r.hal, f));
  CHECK(r.drv.sentFrames() == 1u);
  CHECK(r.link.linkUpCount() == 1u);
}

void test_frag_mtu23_and_tx_full() {
  BleRadioConfig c;
  Rig r(c);
  r.hal.setNegotiatedMtu(23);
  r.hal.setTxSlotsPerTick(4);
  r.hal.setLinkUp(true);
  r.step(5);
  CHECK(r.link.fragPayload() == 19u);
  // 14 片、CPU2 一拍只收 4 片 → 跨 4 拍；RfDriverCore 第 3 拍拿下一幀時上一幀還沒送完 → 忙，丟
  const std::vector<uint8_t> f = pattern(255, 9);
  r.txq.frames.push_back(f);
  r.txq.frames.push_back(pattern(30, 1));  // 第一幀還沒送完 → 忙，丟
  r.step(1);
  CHECK(r.link.txPending());
  CHECK(r.hal.notifyRejectedFull() >= 1u);
  r.step(4);
  CHECK(!r.link.txPending());
  CHECK(r.hal.peerFrames() == 1u);
  CHECK(r.hal.peerFragments() == 14u);
  CHECK(equalLast(r.hal, f));
  CHECK(r.link.busyDrops() == 1u);
  CHECK(r.hal.peerFragErrors() == 0u);
}

void test_link_loss_mid_frame_and_reconnect() {
  Rig r;
  r.hal.setNegotiatedMtu(23);
  r.hal.setTxSlotsPerTick(1);  // 一拍只送一片
  r.hal.setLinkUp(true);
  r.step(5);
  r.txq.frames.push_back(pattern(100, 5));  // 6 片
  r.step(2);
  CHECK(r.link.txPending());
  r.hal.setLinkUp(false);
  r.step(2);
  CHECK(!r.link.txPending());
  CHECK(r.link.txAbortedOnLinkLoss() == 1u);
  CHECK(r.link.linkDownCount() == 1u);
  CHECK(r.hal.peerFrames() == 0u);  // 半幀不交給中心端
  CHECK(r.drv.state() == DrvState::READY);
  // 重連後新幀正常
  r.hal.setTxSlotsPerTick(8);
  r.hal.setLinkUp(true);
  r.step(1);
  const std::vector<uint8_t> f = pattern(60, 11);
  r.txq.frames.push_back(f);
  r.step(3);
  CHECK(r.hal.peerFrames() == 1u);
  CHECK(equalLast(r.hal, f));
  CHECK(r.link.linkUpCount() == 2u);
}

void test_auto_link_flap_counts() {
  Rig r;
  r.hal.setLinkUp(true);
  r.hal.setAutoLink(50, 20);  // 連 50 拍斷一次，20 拍後重連
  r.step(5);
  for (int i = 0; i < 300; ++i) {
    r.txq.frames.push_back(pattern(64, static_cast<uint8_t>(i)));
    r.step(1);
  }
  CHECK(r.drv.state() == DrvState::READY);
  CHECK(r.link.linkDownCount() >= 3u);
  CHECK(r.link.linkUpCount() == r.link.linkDownCount() || r.link.linkUpCount() == r.link.linkDownCount() + 1u);
  CHECK(r.link.noLinkDrops() > 0u);
  CHECK(r.hal.peerFrames() > 0u);
  CHECK(r.hal.peerFragErrors() == 0u);
}

void test_uplink_reassembly() {
  Rig r;
  r.hal.setNegotiatedMtu(23);
  r.hal.setLinkUp(true);
  r.step(5);
  const std::vector<uint8_t> up = pattern(100, 42);
  CHECK(r.hal.peerWriteFrame(up.data(), static_cast<uint32_t>(up.size()), 19));
  r.step(2);
  CHECK(r.sink.uplink == up);
  CHECK(r.link.rxFrames() == 1u);
  CHECK(r.link.rxFragErrors() == 0u);
}

void test_uplink_gap_and_missing_last() {
  Rig r;
  r.hal.setNegotiatedMtu(23);
  r.hal.setLinkUp(true);
  r.step(5);
  // 序號跳號：FIRST(seq0) → 中段(seq2)：整幀作廢
  uint8_t a[5] = {static_cast<uint8_t>(kBleFragFirst | 0u), 1, 2, 3, 4};
  uint8_t b[5] = {static_cast<uint8_t>(kBleFragLast | 2u), 5, 6, 7, 8};
  r.hal.peerWriteFragment(a, 5);
  r.hal.peerWriteFragment(b, 5);
  r.step(1);
  CHECK(r.sink.uplink.empty());
  CHECK(r.link.rxFragErrors() == 1u);
  // 少了 LAST 就來新 FIRST：舊的丟，新的照收
  uint8_t c[3] = {static_cast<uint8_t>(kBleFragFirst | 3u), 9, 9};
  uint8_t d[3] = {static_cast<uint8_t>(kBleFragFirst | kBleFragLast | 4u), 0xAA, 0xBB};
  r.hal.peerWriteFragment(c, 3);
  r.hal.peerWriteFragment(d, 3);
  r.step(1);
  CHECK(r.sink.uplink.size() == 2u);
  CHECK(r.sink.uplink.size() == 2u && r.sink.uplink[0] == 0xAA && r.sink.uplink[1] == 0xBB);
  CHECK(r.link.rxFragErrors() == 2u);
  // 收一半就斷線：半幀丟掉
  uint8_t e[3] = {static_cast<uint8_t>(kBleFragFirst | 0u), 1, 1};
  r.hal.peerWriteFragment(e, 3);
  r.step(1);
  r.hal.setLinkUp(false);
  r.step(1);
  CHECK(r.sink.uplink.size() == 2u);
  CHECK(r.link.rxFragErrors() == 3u);
}

void test_stack_faults() {
  {
    Rig r;
    r.hal.setFault(BleSimFault::START_FAIL);
    r.step(3);
    CHECK(r.drv.state() == DrvState::FAULT);
  }
  {
    Rig r;
    r.hal.setFault(BleSimFault::SELFTEST_FAIL);
    r.step(10);
    CHECK(r.drv.state() == DrvState::FAULT);
    CHECK(r.sink.has(RfEvent::SELFTEST_FAIL));
  }
  {
    Rig r;
    r.hal.setLinkUp(true);
    r.step(5);
    CHECK(r.drv.state() == DrvState::READY);
    r.hal.setFault(BleSimFault::STACK_DEAD);  // CPU2 當掉 → readyDeadline_s 後 FAULT（與 LoRa 一致）
    r.step(100);
    CHECK(r.drv.state() == DrvState::READY);
    r.step(500);
    CHECK(r.drv.state() == DrvState::FAULT);
    CHECK(!r.sink.comReady);
  }
  {
    // FAULT 後外殼重新 start() → 重新 INIT（stackStart 再呼叫一次）
    Rig r;
    r.hal.setFault(BleSimFault::START_FAIL);
    r.step(3);
    CHECK(r.drv.state() == DrvState::FAULT);
    r.hal.setFault(BleSimFault::NONE);
    r.drv.start();
    r.step(5);
    CHECK(r.drv.state() == DrvState::READY);
    CHECK(r.hal.startCalls() == 2u);
  }
}


// Framer 幀原封不動走 BLE：下行 TLM 滿載幀（255 B，MTU 23 → 14 片）＋上行 CMD 幀；CRC 在對端仍對得上
void test_framer_frames_end_to_end() {
  Rig r;
  r.hal.setNegotiatedMtu(23);
  r.hal.setLinkUp(true);
  r.step(5);
  FramerCore fr;
  fr.setFrameMax(255);
  uint8_t payload[255 - FRAMER_HEADER_BYTES - FRAMER_TRAILER_CRC];
  for (size_t i = 0; i < sizeof(payload); ++i) payload[i] = static_cast<uint8_t>(i * 13u);
  FrameRequest q;
  q.type = FrameType::TLM;
  q.source = FrameSource::DS;
  q.apid = 0x010;
  q.seq = 7;
  q.t_s = 1.25;
  q.valid = true;
  q.payload = payload;
  q.payloadLen = static_cast<uint16_t>(sizeof(payload));
  uint8_t buf[300];
  const FrameResult res = fr.build(q, buf, sizeof(buf));
  CHECK(res.ok && res.len == 255u);
  r.txq.frames.push_back(std::vector<uint8_t>(res.data, res.data + res.len));
  r.step(4);
  CHECK(r.hal.peerFrames() == 1u);
  CHECK(r.hal.lastPeerFrameLen() == res.len);
  if (r.hal.lastPeerFrameLen() == res.len) {
    const uint8_t* g = r.hal.lastPeerFrame();
    const uint32_t crc = FramerCore::crc32(g + FRAMER_OFF_LEN, res.len - FRAMER_OFF_LEN - FRAMER_TRAILER_CRC);
    const uint32_t tail = (static_cast<uint32_t>(g[res.len - 4]) << 24) | (static_cast<uint32_t>(g[res.len - 3]) << 16) |
                          (static_cast<uint32_t>(g[res.len - 2]) << 8) | static_cast<uint32_t>(g[res.len - 1]);
    CHECK(crc == tail);
  }
  // 上行：地面 CMD 幀（酬載只是假資料，不含任何金鑰／tag）
  uint8_t cmd[12] = {0x00, 0x01, 0xAB, 0xCD, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  q.type = FrameType::CMD;
  q.source = FrameSource::GROUND;
  q.apid = 0x100;
  q.payload = cmd;
  q.payloadLen = sizeof(cmd);
  const FrameResult up = fr.build(q, buf, sizeof(buf));
  CHECK(up.ok);
  CHECK(r.hal.peerWriteFrame(up.data, static_cast<uint32_t>(up.len), r.link.fragPayload()));
  r.step(2);
  CHECK(r.sink.uplink.size() == up.len);
  CHECK(r.sink.uplink.size() == up.len && std::equal(r.sink.uplink.begin(), r.sink.uplink.end(), up.data));
}

}  // namespace

int BleDriverLegacyMain() {
  g_fail = 0;
  struct T {
    const char* name;
    void (*fn)();
  } tests[] = {
      {"config_clamp", test_config_clamp},
      {"ready_without_link_never_faults", test_ready_without_link_never_faults},
      {"frag_mtu247", test_frag_mtu247},
      {"frag_mtu23_and_tx_full", test_frag_mtu23_and_tx_full},
      {"link_loss_mid_frame_and_reconnect", test_link_loss_mid_frame_and_reconnect},
      {"auto_link_flap_counts", test_auto_link_flap_counts},
      {"uplink_reassembly", test_uplink_reassembly},
      {"uplink_gap_and_missing_last", test_uplink_gap_and_missing_last},
      {"stack_faults", test_stack_faults},
      {"framer_frames_end_to_end", test_framer_frames_end_to_end},
  };
  for (const T& t : tests) {
    const int before = g_fail;
    t.fn();
    std::printf("[%s] %s\n", (g_fail == before) ? "PASS" : "FAIL", t.name);
  }
  std::printf("BleDriver host: %d failure(s)\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}

// 真 F´ UT（gtest，ctest 會跑）；-DHOST_TEST_MAIN 時用 run_host_tests.sh 快速自測
#ifdef HOST_TEST_MAIN
int main() { return BleDriverLegacyMain(); }
#else
#include <gtest/gtest.h>
TEST(Wireless, BleDriverHostSuite) { ASSERT_EQ(0, BleDriverLegacyMain()); }
#endif
