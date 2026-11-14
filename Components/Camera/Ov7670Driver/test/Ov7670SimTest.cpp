// Ov7670SimTest：F´ 單元測試（真 F´ 自動碼型別＋gtest）。涵蓋 Core＋SimDevice 的狀態機路徑。
// 跑法：tools/verify/fprime_verify.sh --ut Components/Camera/CamTypes Components/Camera/Ov7670Driver
#include "Components/Camera/Ov7670Driver/Ov7670Core.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670Gray.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670SimDevice.hpp"

#include <gtest/gtest.h>
#include <cstdio>
#include <cstring>

using namespace Components;

namespace {

struct RecordingSink : IOv7670Sink {
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

  void pullTlm(const Ov7670Core& core) { tlm = core.status(); }  // FPP 自動碼 CamStatus
};

struct Fixture {
  Ov7670SimDevice sim{};
  RecordingSink sink{};
  Ov7670Core core{};
  Fixture() {
    core.configure(&sim, &sim, &sink);
    // 既有案例測的是每拍一幀的狀態機路徑：關掉 1 fps 上限（上限本身由 testOneFpsLimit 測）。
    Ov7670Params p{};
    p.framePeriodTicks = 0;
    core.setParams(p);
  }
};

void run(Ov7670Core& core, Ov7670SimDevice& sim, uint32_t tickHz, uint32_t ticks) {
  for (uint32_t i = 0; i < ticks; ++i) {
    sim.advance();
    core.tick(tickHz);
  }
}

bool runUntil(Ov7670Core& core, Ov7670SimDevice& sim, uint32_t tickHz, CamState target,
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
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  const uint32_t sinkBefore = f.sink.frameEvents;
  run(f.core, f.sim, 100u, 5u);
  ASSERT_TRUE(f.sink.frameEvents == sinkBefore + 5u);
  ASSERT_TRUE(f.sink.lastLen == Ov7670SimDevice::kSimFrameBytes);
  ASSERT_TRUE(f.core.frameCount() == 5u);
  f.sink.pullTlm(f.core);
  ASSERT_TRUE(f.sink.tlm.get_camState() == CamState::READY);
  ASSERT_TRUE(f.sink.tlm.get_frameCount() == f.core.frameCount());
  ASSERT_TRUE(f.sink.tlm.get_fault() == CamFault::NONE);
  ASSERT_TRUE(f.sink.tlm.get_model() == CamModel::OV7670);
  ASSERT_TRUE(f.sink.tlm.get_role() == CamRole::US_CAM);
}

void testCaptureRejectedWhenNotReady() {
  Fixture f;
  ASSERT_TRUE(!f.core.captureStart());
  f.core.start();
  ASSERT_TRUE(f.core.state() == CamState::INIT);
  ASSERT_TRUE(!f.core.captureStart());
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  f.core.captureStop();
  ASSERT_TRUE(!f.core.capturing());
  ASSERT_TRUE(f.core.captureStart());
}

void testBadChipId() {
  Fixture f;
  f.sim.setFault(Ov7670SimFault::BAD_CHIP_ID);
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::FAULT, 64u));
  ASSERT_TRUE(f.core.fault() == CamFault::ID_MISMATCH);
  ASSERT_TRUE(f.sink.lastFault == CamFault::ID_MISMATCH);
}

void testTypeRoleMismatch() {
  Fixture f;
  Ov7670Params p{};
  p.model = CamModel::ESP32_CAM;
  p.role = CamRole::US_CAM;
  f.core.setParams(p);
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::FAULT, 64u));
  ASSERT_TRUE(f.core.fault() == CamFault::TYPE_ROLE_MISMATCH);

  Fixture g;
  Ov7670Params pg{};
  pg.model = CamModel::OV7670;
  pg.role = CamRole::DS_CAM;
  g.core.setParams(pg);
  g.core.start();
  ASSERT_TRUE(runUntil(g.core, g.sim, 100u, CamState::FAULT, 64u));
  ASSERT_TRUE(g.core.fault() == CamFault::TYPE_ROLE_MISMATCH);
}

void testBusTimeout() {
  Fixture f;
  f.core.start();
  run(f.core, f.sim, 100u, 1u);
  f.sim.setFault(Ov7670SimFault::BUS_DOWN);
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::FAULT, 16u));
  ASSERT_TRUE(f.core.fault() == CamFault::BUS_TIMEOUT);
}

void testFrameTimeout() {
  Fixture f;
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  f.sim.setFrameEveryN(0xFFFFFFFFu);
  f.sim.setFrameAvailable(false);
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::FAULT, 128u));
  ASSERT_TRUE(f.core.fault() == CamFault::FRAME_TIMEOUT);
}

void testStopResetRehandshake() {
  Fixture f;
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  run(f.core, f.sim, 100u, 3u);
  ASSERT_TRUE(f.core.frameCount() >= 3u);

  f.core.stop();
  ASSERT_TRUE(f.core.state() == CamState::OFF);
  ASSERT_TRUE(f.core.frameCount() >= 3u);

  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));

  f.core.reset();
  ASSERT_TRUE(f.core.state() == CamState::OFF);
  ASSERT_TRUE(f.core.fault() == CamFault::NONE);
  ASSERT_TRUE(f.core.frameCount() == 0u);
  ASSERT_TRUE(!f.core.recoverLatched());

  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.fault() == CamFault::NONE);
  ASSERT_TRUE(f.core.lastChipId() == Ov7670Regs::EXPECTED_CHIP_ID);
}

void testInitChunking() {
  // 初始化表 13 筆；第 0 筆是軟重置，寫完那一拍就停（下一拍才寫其餘）
  Fixture f;
  Ov7670Params p{};
  p.entriesPerTick = 8;
  f.core.setParams(p);
  f.core.start();

  f.sim.advance();
  f.core.tick(100u);
  ASSERT_TRUE(f.core.initIndex() == 1u);  // 只寫了 COM7=0x80

  f.sim.advance();
  f.core.tick(100u);
  ASSERT_TRUE(f.core.initIndex() == 9u);

  f.sim.advance();
  f.core.tick(100u);
  ASSERT_TRUE(f.core.initIndex() == 13u);
  ASSERT_TRUE(f.core.state() == CamState::SELFTEST);

  f.core.reset();
  Ov7670Params p2{};
  p2.entriesPerTick = 4;
  f.core.setParams(p2);
  f.core.start();
  const uint32_t expect[] = {1u, 5u, 9u, 13u};
  for (uint32_t e : expect) {
    f.sim.advance();
    f.core.tick(100u);
    ASSERT_TRUE(f.core.initIndex() == e);
  }
  ASSERT_TRUE(f.core.state() == CamState::SELFTEST);
}

void testRecoveryAfterBusFault() {
  Fixture f;
  f.core.start();
  run(f.core, f.sim, 100u, 1u);
  f.sim.setFault(Ov7670SimFault::BUS_DOWN);
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::FAULT, 16u));
  ASSERT_TRUE(f.core.fault() == CamFault::BUS_TIMEOUT);

  f.sim.setFault(Ov7670SimFault::NONE);
  f.core.reset();
  ASSERT_TRUE(f.core.state() == CamState::OFF);
  ASSERT_TRUE(f.core.fault() == CamFault::NONE);

  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
}

void testParamFrameTimeoutAnd1Hz() {
  Fixture f;
  Ov7670Params p{};
  p.frameTimeoutTicks = 5;
  f.core.setParams(p);
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  f.sim.setFrameEveryN(0xFFFFFFFFu);
  f.sim.setFrameAvailable(false);
  for (uint32_t i = 0; i < 5; ++i) {
    f.sim.advance();
    f.core.tick(100u);
  }
  ASSERT_TRUE(f.core.state() == CamState::FAULT);
  ASSERT_TRUE(f.core.fault() == CamFault::FRAME_TIMEOUT);

  Fixture g;
  Ov7670Params pg{};
  pg.frameTimeoutTicks = 3;
  g.core.setParams(pg);
  g.core.start();
  ASSERT_TRUE(runUntil(g.core, g.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(g.core.captureStart());
  g.sim.setFrameEveryN(0xFFFFFFFFu);
  g.sim.setFrameAvailable(false);
  for (uint32_t i = 0; i < 10; ++i) {
    g.sim.advance();
    g.core.tick(1u);
  }
  ASSERT_TRUE(g.core.state() == CamState::READY);
  ASSERT_TRUE(g.core.tick(1000u) == false);
  ASSERT_TRUE(g.core.state() == CamState::READY);
}

void testTelemetryHasNoImage() {
  Fixture f;
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  run(f.core, f.sim, 100u, 2u);
  ASSERT_TRUE(f.sink.frameEvents >= 2u);
  ASSERT_TRUE(f.sink.lastData[0] == 0xA0);
  f.sink.pullTlm(f.core);
  const CamStatus s = f.sink.tlm;
  ASSERT_TRUE(s.get_frameCount() == f.core.frameCount());
  ASSERT_TRUE(s.get_model() == CamModel::OV7670);
  ASSERT_TRUE(s.get_role() == CamRole::US_CAM);
  (void)s;
}

// Binding #1: FAULT → 1 Hz every K ticks auto INIT, emit RecoverAttempt, up to R; then latch
void testFaultRecoverKR() {
  Fixture f;
  Ov7670Params p{};
  p.faultRecoverK = 5;
  p.faultRecoverR = 3;
  f.core.setParams(p);
  f.core.start();
  run(f.core, f.sim, 100u, 1u);
  f.sim.setFault(Ov7670SimFault::BUS_DOWN);
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::FAULT, 16u));

  // Keep bus down so each recover fails again
  for (uint32_t attempt = 1; attempt <= 3; ++attempt) {
    const uint32_t before = f.sink.recoverEvents;
    for (uint32_t i = 0; i < 5; ++i) {
      f.sim.advance();
      f.core.tick(1u);
    }
    ASSERT_TRUE(f.sink.recoverEvents == before + 1);
    ASSERT_TRUE(f.sink.lastRecoverAttempt == attempt);
    ASSERT_TRUE(f.sink.lastRecoverLimit == 3);
    // After recover → INIT, bus still down → FAULT again
    ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::FAULT, 32u));
  }
  // Next K ticks: latch, no more RecoverAttempt
  const uint32_t locked = f.sink.recoverEvents;
  for (uint32_t i = 0; i < 10; ++i) {
    f.sim.advance();
    f.core.tick(1u);
  }
  ASSERT_TRUE(f.sink.recoverEvents == locked);
  ASSERT_TRUE(f.core.recoverLatched());
  ASSERT_TRUE(f.core.state() == CamState::FAULT);

  // Only RESET clears latch
  f.sim.setFault(Ov7670SimFault::NONE);
  f.core.reset();
  ASSERT_TRUE(!f.core.recoverLatched());
  ASSERT_TRUE(f.core.recoverAttempts() == 0);
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
}

// Binding #3 (Core side): rejected onFrame must not increment frameCount
void testRejectedFrameNotCounted() {
  Fixture f;
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  f.sink.acceptFrames = false;
  run(f.core, f.sim, 100u, 3u);
  ASSERT_TRUE(f.sink.frameEvents == 3u);
  ASSERT_TRUE(f.core.frameCount() == 0u);
  f.sink.acceptFrames = true;
  run(f.core, f.sim, 100u, 2u);
  ASSERT_TRUE(f.core.frameCount() == 2u);
}

// Edward 2026-11-14：上級 1 fps。感測器每拍都出幀，預設 FRAME_PERIOD_TICKS＝100 → 每 100 拍只轉存一幀，
// 丟掉的幀不算斷訊（不會 FRAME_TIMEOUT）。
void testOneFpsLimit() {
  Fixture f;
  f.core.setParams(Ov7670Params{});  // 預設值：framePeriodTicks＝100
  ASSERT_EQ(f.core.params().framePeriodTicks, 100u);
  f.core.start();
  ASSERT_TRUE(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  ASSERT_TRUE(f.core.captureStart());
  run(f.core, f.sim, 100u, 1u);
  ASSERT_EQ(f.core.frameCount(), 1u);  // 開始擷取後第一幀立刻轉存
  run(f.core, f.sim, 100u, 99u);
  ASSERT_EQ(f.core.frameCount(), 1u);  // 還沒滿 1 s
  run(f.core, f.sim, 100u, 1u);
  ASSERT_EQ(f.core.frameCount(), 2u);  // 第 101 拍＝1 s 後
  run(f.core, f.sim, 100u, 199u);
  ASSERT_EQ(f.core.frameCount(), 3u);  // 3 s 內 3 幀（第 1、101、201 拍）
  ASSERT_TRUE(f.core.state() == CamState::READY);
  ASSERT_EQ(f.sink.frameEvents, 3u);
}

// 灰階：YUYV（Y0 U0 Y1 V0 …）只取偶數位元組；分塊起點、尾端截斷都要對。
void testGrayPackY() {
  uint8_t yuyv[12];
  for (uint32_t i = 0; i < 12; ++i) yuyv[i] = static_cast<uint8_t>(i % 2 == 0 ? 100 + i / 2 : 0xEE);
  ASSERT_EQ(ov7670GrayPixels(12u), 6u);
  uint8_t dst[8]{};
  ASSERT_EQ(ov7670PackY(yuyv, 12u, 0u, dst, 4u), 4u);
  ASSERT_EQ(dst[0], 100u);
  ASSERT_EQ(dst[3], 103u);
  ASSERT_EQ(ov7670PackY(yuyv, 12u, 4u, dst, 8u), 2u);  // 只剩 2 個像素
  ASSERT_EQ(dst[0], 104u);
  ASSERT_EQ(dst[1], 105u);
  ASSERT_EQ(ov7670PackY(yuyv, 12u, 6u, dst, 8u), 0u);
  ASSERT_EQ(ov7670PackY(nullptr, 12u, 0u, dst, 8u), 0u);
  // 實際尺寸：QCIF YUYV 50,688 B → 灰階 25,344 B（2 KB 分塊＝13 塊）
  ASSERT_EQ(ov7670GrayPixels(Ov7670Regs::RAW_FRAME_BYTES), Ov7670Regs::FRAME_BYTES);
  ASSERT_EQ((Ov7670Regs::FRAME_BYTES + 2047u) / 2048u, 13u);
}

}  // namespace

TEST(Ov7670Core, NormalPath) { testNormalPath(); }
TEST(Ov7670Core, CaptureRejectedWhenNotReady) { testCaptureRejectedWhenNotReady(); }
TEST(Ov7670Core, BadChipId) { testBadChipId(); }
TEST(Ov7670Core, TypeRoleMismatch) { testTypeRoleMismatch(); }
TEST(Ov7670Core, BusTimeout) { testBusTimeout(); }
TEST(Ov7670Core, FrameTimeout) { testFrameTimeout(); }
TEST(Ov7670Core, StopResetRehandshake) { testStopResetRehandshake(); }
TEST(Ov7670Core, InitChunking) { testInitChunking(); }
TEST(Ov7670Core, RecoveryAfterBusFault) { testRecoveryAfterBusFault(); }
TEST(Ov7670Core, ParamFrameTimeoutAnd1Hz) { testParamFrameTimeoutAnd1Hz(); }
TEST(Ov7670Core, TelemetryHasNoImage) { testTelemetryHasNoImage(); }
TEST(Ov7670Core, FaultRecoverKR) { testFaultRecoverKR(); }
TEST(Ov7670Core, RejectedFrameNotCounted) { testRejectedFrameNotCounted(); }
TEST(Ov7670Core, OneFpsLimit) { testOneFpsLimit(); }
TEST(Ov7670Core, GrayPackY) { testGrayPackY(); }
