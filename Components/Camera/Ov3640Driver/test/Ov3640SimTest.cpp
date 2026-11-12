// Ov3640SimTest：host 單元測試（無 F´）。涵蓋需求列舉的路徑。
#include "Components/Camera/Ov3640Driver/Ov3640Core.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640SimDevice.hpp"

#include <cassert>
#include <cstdio>
#include <cstring>

using namespace Components;

namespace {

struct RecordingSink : IOv3640Sink {
  static constexpr uint32_t kMaxBytes = 64;

  uint32_t frameEvents = 0;
  uint32_t lastIndex = 0;
  uint32_t lastLen = 0;
  uint8_t lastData[kMaxBytes]{};
  uint32_t stateChanges = 0;
  uint32_t faultEvents = 0;
  uint32_t recoverEvents = 0;
  uint32_t lastRecoverAttempt = 0;
  uint32_t lastRecoverLimit = 0;
  CamFault lastFault = CamFault::NONE;
  bool acceptFrames = true;

  CamStatus tlm{};

  bool onFrame(const uint8_t* data, uint32_t length, uint32_t frameIndex) override {
    ++frameEvents;
    lastIndex = frameIndex;
    lastLen = length;
    const uint32_t n = length < kMaxBytes ? length : kMaxBytes;
    if (data && n > 0) {
      std::memcpy(lastData, data, n);
    }
    return acceptFrames;
  }
  void onStateChange(CamState, CamState) override { ++stateChanges; }
  void onFault(CamFault code, uint32_t) override {
    ++faultEvents;
    lastFault = code;
  }
  void onRecoverAttempt(uint32_t attempt, uint32_t limit) override {
    ++recoverEvents;
    lastRecoverAttempt = attempt;
    lastRecoverLimit = limit;
  }

  void pullTlm(const Ov3640Core& core) { tlm = core.status(); }
};

struct Fixture {
  Ov3640SimDevice sim{};
  RecordingSink sink{};
  Ov3640Core core{};
  Fixture() { core.configure(&sim, &sim, &sink); }
};

void run(Ov3640Core& core, Ov3640SimDevice& sim, uint32_t tickHz, uint32_t ticks) {
  for (uint32_t i = 0; i < ticks; ++i) {
    sim.advance();
    core.tick(tickHz);
  }
}

bool runUntil(Ov3640Core& core, Ov3640SimDevice& sim, uint32_t tickHz, CamState target,
              uint32_t maxTicks) {
  for (uint32_t i = 0; i < maxTicks; ++i) {
    if (core.state() == target) return true;
    sim.advance();
    core.tick(tickHz);
  }
  return core.state() == target;
}

void testNormalPath() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.captureStart());
  const uint32_t sinkBefore = f.sink.frameEvents;
  run(f.core, f.sim, 100u, 5u);
  assert(f.sink.frameEvents == sinkBefore + 5u);
  assert(f.sink.lastLen == Ov3640SimDevice::kSimFrameBytes);
  assert(f.core.frameCount() == 5u);
  f.sink.pullTlm(f.core);
  assert(f.sink.tlm.get_camState() == CamState::READY);
  assert(f.sink.tlm.get_frameCount() == f.core.frameCount());
  assert(f.sink.tlm.get_fault() == CamFault::NONE);
  assert(f.sink.tlm.get_model() == CamModel::OV3640);
  assert(f.sink.tlm.get_role() == CamRole::US_CAM);
}

void testCaptureRejectedWhenNotReady() {
  Fixture f;
  assert(!f.core.captureStart());
  f.core.start();
  assert(f.core.state() == CamState::INIT);
  assert(!f.core.captureStart());
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.captureStart());
  f.core.captureStop();
  assert(!f.core.capturing());
  assert(f.core.captureStart());
}

void testBadChipId() {
  Fixture f;
  f.sim.setFault(Ov3640SimFault::BAD_CHIP_ID);
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 64u));
  assert(f.core.fault() == CamFault::ID_MISMATCH);
  assert(f.sink.lastFault == CamFault::ID_MISMATCH);
}

void testTypeRoleMismatch() {
  Fixture f;
  Ov3640Params p{};
  p.model = CamModel::ESP32_CAM;
  p.role = CamRole::US_CAM;
  f.core.setParams(p);
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 64u));
  assert(f.core.fault() == CamFault::TYPE_ROLE_MISMATCH);

  Fixture g;
  Ov3640Params pg{};
  pg.model = CamModel::OV3640;
  pg.role = CamRole::DS_CAM;
  g.core.setParams(pg);
  g.core.start();
  assert(runUntil(g.core, g.sim, 100u, CamState::FAULT, 64u));
  assert(g.core.fault() == CamFault::TYPE_ROLE_MISMATCH);
}

void testBusTimeout() {
  Fixture f;
  f.core.start();
  run(f.core, f.sim, 100u, 1u);
  f.sim.setFault(Ov3640SimFault::BUS_DOWN);
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 16u));
  assert(f.core.fault() == CamFault::BUS_TIMEOUT);
}

void testFrameTimeout() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.captureStart());
  f.sim.setFrameEveryN(0xFFFFFFFFu);
  f.sim.setFrameAvailable(false);
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 128u));
  assert(f.core.fault() == CamFault::FRAME_TIMEOUT);
}

void testStopResetRehandshake() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.captureStart());
  run(f.core, f.sim, 100u, 3u);
  assert(f.core.frameCount() >= 3u);

  f.core.stop();
  assert(f.core.state() == CamState::OFF);
  assert(f.core.frameCount() >= 3u);

  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));

  f.core.reset();
  assert(f.core.state() == CamState::OFF);
  assert(f.core.fault() == CamFault::NONE);
  assert(f.core.frameCount() == 0u);
  assert(!f.core.recoverLatched());

  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.fault() == CamFault::NONE);
  assert(f.core.lastChipId() == 0x364C);
}

void testInitChunking() {
  Fixture f;
  Ov3640Params p{};
  p.entriesPerTick = 8;
  f.core.setParams(p);
  f.core.start();

  f.sim.advance();
  f.core.tick(100u);
  assert(f.core.initIndex() == 8u);

  f.sim.advance();
  f.core.tick(100u);
  assert(f.core.initIndex() == 12u);
  assert(f.core.state() == CamState::SELFTEST);

  f.core.reset();
  Ov3640Params p2{};
  p2.entriesPerTick = 4;
  f.core.setParams(p2);
  f.core.start();
  f.sim.advance();
  f.core.tick(100u);
  assert(f.core.initIndex() == 4u);
  f.sim.advance();
  f.core.tick(100u);
  assert(f.core.initIndex() == 8u);
  f.sim.advance();
  f.core.tick(100u);
  assert(f.core.initIndex() == 12u);
  assert(f.core.state() == CamState::SELFTEST);
}

void testRecoveryAfterBusFault() {
  Fixture f;
  f.core.start();
  run(f.core, f.sim, 100u, 1u);
  f.sim.setFault(Ov3640SimFault::BUS_DOWN);
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 16u));
  assert(f.core.fault() == CamFault::BUS_TIMEOUT);

  f.sim.setFault(Ov3640SimFault::NONE);
  f.core.reset();
  assert(f.core.state() == CamState::OFF);
  assert(f.core.fault() == CamFault::NONE);

  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
}

void testParamFrameTimeoutAnd1Hz() {
  Fixture f;
  Ov3640Params p{};
  p.frameTimeoutTicks = 5;
  f.core.setParams(p);
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.captureStart());
  f.sim.setFrameEveryN(0xFFFFFFFFu);
  f.sim.setFrameAvailable(false);
  for (uint32_t i = 0; i < 5; ++i) {
    f.sim.advance();
    f.core.tick(100u);
  }
  assert(f.core.state() == CamState::FAULT);
  assert(f.core.fault() == CamFault::FRAME_TIMEOUT);

  Fixture g;
  Ov3640Params pg{};
  pg.frameTimeoutTicks = 3;
  g.core.setParams(pg);
  g.core.start();
  assert(runUntil(g.core, g.sim, 100u, CamState::READY, 64u));
  assert(g.core.captureStart());
  g.sim.setFrameEveryN(0xFFFFFFFFu);
  g.sim.setFrameAvailable(false);
  for (uint32_t i = 0; i < 10; ++i) {
    g.sim.advance();
    g.core.tick(1u);
  }
  assert(g.core.state() == CamState::READY);
  assert(g.core.tick(1000u) == false);
  assert(g.core.state() == CamState::READY);
}

void testTelemetryHasNoImage() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.captureStart());
  run(f.core, f.sim, 100u, 2u);
  assert(f.sink.frameEvents >= 2u);
  assert(f.sink.lastData[0] == 0xA0);
  f.sink.pullTlm(f.core);
  const CamStatus s = f.sink.tlm;
  assert(s.get_frameCount() == f.core.frameCount());
  assert(s.get_model() == CamModel::OV3640);
  assert(s.get_role() == CamRole::US_CAM);
  (void)s;
}

// Binding #1: FAULT → 1 Hz every K ticks auto INIT, emit RecoverAttempt, up to R; then latch
void testFaultRecoverKR() {
  Fixture f;
  Ov3640Params p{};
  p.faultRecoverK = 5;
  p.faultRecoverR = 3;
  f.core.setParams(p);
  f.core.start();
  run(f.core, f.sim, 100u, 1u);
  f.sim.setFault(Ov3640SimFault::BUS_DOWN);
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 16u));

  // Keep bus down so each recover fails again
  for (uint32_t attempt = 1; attempt <= 3; ++attempt) {
    const uint32_t before = f.sink.recoverEvents;
    for (uint32_t i = 0; i < 5; ++i) {
      f.sim.advance();
      f.core.tick(1u);
    }
    assert(f.sink.recoverEvents == before + 1);
    assert(f.sink.lastRecoverAttempt == attempt);
    assert(f.sink.lastRecoverLimit == 3);
    // After recover → INIT, bus still down → FAULT again
    assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 32u));
  }
  // Next K ticks: latch, no more RecoverAttempt
  const uint32_t locked = f.sink.recoverEvents;
  for (uint32_t i = 0; i < 10; ++i) {
    f.sim.advance();
    f.core.tick(1u);
  }
  assert(f.sink.recoverEvents == locked);
  assert(f.core.recoverLatched());
  assert(f.core.state() == CamState::FAULT);

  // Only RESET clears latch
  f.sim.setFault(Ov3640SimFault::NONE);
  f.core.reset();
  assert(!f.core.recoverLatched());
  assert(f.core.recoverAttempts() == 0);
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
}

// Binding #3 (Core side): rejected onFrame must not increment frameCount
void testRejectedFrameNotCounted() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.captureStart());
  f.sink.acceptFrames = false;
  run(f.core, f.sim, 100u, 3u);
  assert(f.sink.frameEvents == 3u);
  assert(f.core.frameCount() == 0u);
  f.sink.acceptFrames = true;
  run(f.core, f.sim, 100u, 2u);
  assert(f.core.frameCount() == 2u);
}

}  // namespace

int main() {
  testNormalPath();
  testCaptureRejectedWhenNotReady();
  testBadChipId();
  testTypeRoleMismatch();
  testBusTimeout();
  testFrameTimeout();
  testStopResetRehandshake();
  testInitChunking();
  testRecoveryAfterBusFault();
  testParamFrameTimeoutAnd1Hz();
  testTelemetryHasNoImage();
  testFaultRecoverKR();
  testRejectedFrameNotCounted();
  std::printf("Ov3640SimTest: all 13 tests passed\n");
  std::printf("FPP declares model=OV3640 role=US_CAM (see Ov3640Driver.fpp constants)\n");
  return 0;
}
