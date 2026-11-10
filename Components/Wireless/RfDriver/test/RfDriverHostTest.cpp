// RfDriverHostTest：§12.1 測試條件
#include "Components/Wireless/RfDriver/RfDriverCore.hpp"
#include "Components/Wireless/RfDriverSim/RfDriverSimDevice.hpp"
#include <cassert>
#include <cstdio>
#include <vector>

using namespace Components;

namespace {

class TestSink : public IRfSink {
 public:
  void onStateChange(DrvState, DrvState to) override { stateChanges.push_back((int)to); }
  void onEvent(RfEvent ev, uint32_t) override { events.push_back((int)ev); }
  void onUplink(const uint8_t*, uint32_t len) override { uplinkLen += len; }
  void onComStatus(bool ready) override { lastComReady = ready; }
  std::vector<int> stateChanges;
  std::vector<int> events;
  uint32_t uplinkLen = 0;
  bool lastComReady = false;
};

class TestTxQueue : public IRfTxQueue {
 public:
  uint32_t popFrame(uint8_t* buf, uint32_t maxLen) override {
    if (frames.empty()) return 0;
    auto f = frames.front();
    frames.erase(frames.begin());
    if (f.size() > maxLen) return 0;
    for (size_t i = 0; i < f.size(); ++i) buf[i] = f[i];
    return (uint32_t)f.size();
  }
  void push(const std::vector<uint8_t>& f) { frames.push_back(f); }
  std::vector<std::vector<uint8_t>> frames;
};

bool hasEvent(const std::vector<int>& v, RfEvent e) {
  for (int x : v) if (x == (int)e) return true;
  return false;
}

void advanceToReady(RfDriverCore& drv, RfDriverSimDevice& bus, double& t) {
  for (int i = 0; i < 20; ++i) {
    bus.advance();
    drv.tick(t);
    t += 0.001;
    if (drv.state() == DrvState::READY) return;
  }
}

void test_handshake_and_phase_timeout() {
  RfDriverSimDevice bus;
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  drv.configure(RfParams{});
  drv.start();
  double t = 0.0;
  advanceToReady(drv, bus, t);
  assert(drv.state() == DrvState::READY);
  assert(hasEvent(sink.events, RfEvent::READY));

  // NOT_READY past readyDeadline → FAULT
  RfDriverSimDevice bus2;
  bus2.setFault(RfSimFault::NOT_READY);
  TestSink sink2;
  TestTxQueue txq2;
  RfDriverCore drv2(bus2, sink2, txq2);
  RfParams p2;
  p2.readyDeadline_s = 0.01;
  drv2.configure(p2);
  drv2.start();
  double t2 = 0.0;
  for (int i = 0; i < 40; ++i) {
    bus2.advance();
    drv2.tick(t2);
    t2 += 0.001;
  }
  assert(drv2.state() == DrvState::FAULT);
  assert(hasEvent(sink2.events, RfEvent::PHASE_TIMEOUT));
}

void test_selftest_ok() {
  RfDriverSimDevice bus;
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  drv.start();
  double t = 0.0;
  advanceToReady(drv, bus, t);
  assert(hasEvent(sink.events, RfEvent::SELFTEST_OK));
  assert(hasEvent(sink.events, RfEvent::INIT_OK));
}

void test_selftest_fail() {
  RfDriverSimDevice bus;
  bus.setFault(RfSimFault::SELFTEST_FAIL);
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  drv.start();
  double t = 0.0;
  for (int i = 0; i < 10; ++i) {
    bus.advance();
    drv.tick(t);
    t += 0.001;
  }
  assert(drv.state() == DrvState::FAULT);
  assert(hasEvent(sink.events, RfEvent::SELFTEST_FAIL));
}

void test_tx_timeout() {
  RfDriverSimDevice bus;
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  RfParams p;
  p.txTimeout_s = 0.002;
  p.txTimeoutLimit = 100;  // don't FAULT yet
  drv.configure(p);
  drv.start();
  double t = 0.0;
  advanceToReady(drv, bus, t);
  txq.push({0x01, 0x02, 0x03, 0x04});
  bus.advance();
  drv.tick(t);  // start TX
  // jump past timeout without completing
  t += 0.010;
  bus.advance();
  drv.tick(t);
  assert(drv.txTimeoutCount() >= 1);
  assert(hasEvent(sink.events, RfEvent::TX_TIMEOUT));
}

void test_consec_tx_timeout_fault() {
  RfDriverSimDevice bus;
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  RfParams p;
  p.txTimeout_s = 0.001;
  p.txTimeoutLimit = 3;
  drv.configure(p);
  drv.start();
  double t = 0.0;
  advanceToReady(drv, bus, t);
  for (int i = 0; i < 3; ++i) {
    txq.push({0xAA});
    bus.advance();
    drv.tick(t);
    t += 0.010;  // exceed timeout
    bus.advance();
    drv.tick(t);
    t += 0.001;
  }
  assert(drv.state() == DrvState::FAULT);
  assert(drv.consecutiveTxTimeouts() >= 3);
}

void test_sim_drop_rate() {
  RfDriverSimDevice bus;
  bus.setDropRate(0.5);
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  RfParams p;
  p.txTimeout_s = 100.0;
  drv.configure(p);
  drv.start();
  double t = 0.0;
  advanceToReady(drv, bus, t);
  const int N = 2000;
  for (int i = 0; i < N; ++i) {
    txq.push({0xAA});
    bus.advance();
    drv.tick(t);
    t += 0.0001;
    // complete in-flight
    bus.advance();
    drv.tick(t);
    t += 0.0001;
  }
  const double total = (double)bus.framesWritten() + (double)bus.framesDropped();
  assert(total > 0.0);
  const double observed = (double)bus.framesDropped() / total;
  double err = observed - 0.5;
  if (err < 0) err = -err;
  assert(err < 0.05);
}

void test_interface_and_uplink() {
  RfDriverSimDevice bus;
  TestSink sink;
  TestTxQueue txq;
  IRfBus& busRef = bus;
  IRfSink& sinkRef = sink;
  IRfTxQueue& txqRef = txq;
  RfDriverCore drv(busRef, sinkRef, txqRef);
  drv.start();
  double t = 0.0;
  advanceToReady(drv, bus, t);
  uint8_t up[3] = {0x11, 0x22, 0x33};
  bus.pushUplink(up, 3);
  bus.advance();
  drv.tick(t);
  assert(sink.uplinkLen == 3);
}

void test_sent_frames() {
  RfDriverSimDevice bus;
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  RfParams p;
  p.txTimeout_s = 100.0;
  drv.configure(p);
  drv.start();
  double t = 0.0;
  advanceToReady(drv, bus, t);
  const uint32_t K = 5;
  for (uint32_t i = 0; i < K; ++i) {
    txq.push({0xDE, 0xAD, 0xBE, 0xEF, (uint8_t)i});
    bus.advance();
    drv.tick(t);
    t += 0.001;
    bus.advance();
    drv.tick(t);  // complete
    t += 0.001;
  }
  assert(drv.sentFrames() == K);
  assert(bus.framesWritten() == K);
}

void test_init_deadline() {
  // init always succeeds quickly with sim; use INIT_FAIL
  RfDriverSimDevice bus;
  bus.setFault(RfSimFault::INIT_FAIL);
  TestSink sink;
  TestTxQueue txq;
  RfDriverCore drv(bus, sink, txq);
  drv.start();
  double t = 0.0;
  bus.advance();
  drv.tick(t);
  assert(drv.state() == DrvState::FAULT);
}

}  // namespace

static int RfDriverLegacyMain() {
  test_handshake_and_phase_timeout();
  test_selftest_ok();
  test_selftest_fail();
  test_tx_timeout();
  test_consec_tx_timeout_fault();
  test_sim_drop_rate();
  test_interface_and_uplink();
  test_sent_frames();
  test_init_deadline();
  std::printf("RfDriverHostTest: all 9 tests passed\n");
  return 0;
}

// 真 F´ UT（gtest，ctest 會跑）；-DHOST_TEST_MAIN 時仍可用舊的 run_host_tests.sh 快速自測
#ifdef HOST_TEST_MAIN
int main() { return RfDriverLegacyMain(); }
#else
#include <gtest/gtest.h>
TEST(Wireless, RfDriverHostSuite) { ASSERT_EQ(0, RfDriverLegacyMain()); }
#endif
