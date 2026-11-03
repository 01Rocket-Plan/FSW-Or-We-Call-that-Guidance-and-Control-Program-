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
  CamFault lastFault = CamFault::NONE;

  // 遙測鏡像：只允許 CamStatus 欄位（測試「無影像」）
  CamStatus tlm{};

  void onFrame(const uint8_t* data, uint32_t length, uint32_t frameIndex) override {
    ++frameEvents;
    lastIndex = frameIndex;
    lastLen = length;
    const uint32_t n = length < kMaxBytes ? length : kMaxBytes;
    if (data && n > 0) {
      std::memcpy(lastData, data, n);
    }
  }
  void onStateChange(CamState, CamState) override { ++stateChanges; }
  void onFault(CamFault code, uint32_t) override {
    ++faultEvents;
    lastFault = code;
  }

  void pullTlm(const Ov3640Core& core) {
    tlm = core.status();
  }
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

// 1) 正常路徑：OFF→READY，frameCount 遞增，影像只出現在 sink
void testNormalPath() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.fault() == CamFault::NONE);
  assert(f.core.lastChipId() == 0x364C);
  assert(f.core.status().model == CamModel::OV3640);
  assert(f.core.status().role == CamRole::US_CAM);

  const uint32_t before = f.core.frameCount();
  const uint32_t sinkBefore = f.sink.frameEvents;
  run(f.core, f.sim, 100u, 5u);
  assert(f.core.frameCount() == before + 5u);
  assert(f.sink.frameEvents == sinkBefore + 5u);
  assert(f.sink.lastLen == Ov3640SimDevice::kSimFrameBytes);

  f.sink.pullTlm(f.core);
  // 遙測只有 CamStatus 五個邏輯欄位，不含影像指標／緩衝
  assert(f.sink.tlm.state == CamState::READY);
  assert(f.sink.tlm.frameCount == f.core.frameCount());
  // CamStatus 僅狀態欄位（可有 padding）；不得也不會承載影像緩衝
  assert(f.sink.tlm.fault == CamFault::NONE);
  assert(f.sink.tlm.model == CamModel::OV3640);
  assert(f.sink.tlm.role == CamRole::US_CAM);
}

// 2) chip ID 錯誤 → ID_MISMATCH
void testBadChipId() {
  Fixture f;
  f.sim.setFault(Ov3640SimFault::BAD_CHIP_ID);
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 64u));
  assert(f.core.fault() == CamFault::ID_MISMATCH);
  assert(f.sink.lastFault == CamFault::ID_MISMATCH);
}

// 3) 型號／角色不符 → TYPE_ROLE_MISMATCH
void testTypeRoleMismatch() {
  {
    Fixture f;
    Ov3640Params p{};
    p.model = CamModel::ESP32_CAM;
    p.role = CamRole::US_CAM;
    f.core.setParams(p);
    f.core.start();
    assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 64u));
    assert(f.core.fault() == CamFault::TYPE_ROLE_MISMATCH);
  }
  {
    Fixture f;
    Ov3640Params p{};
    p.model = CamModel::OV3640;
    p.role = CamRole::DS_CAM;
    f.core.setParams(p);
    f.core.start();
    assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 64u));
    assert(f.core.fault() == CamFault::TYPE_ROLE_MISMATCH);
  }
}

// 4) 匯流排逾時 N=3 → BUS_TIMEOUT
void testBusTimeout() {
  Fixture f;
  f.core.start();
  run(f.core, f.sim, 100u, 1u);
  assert(f.core.state() == CamState::INIT);
  f.sim.setFault(Ov3640SimFault::BUS_DOWN);
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 16u));
  assert(f.core.fault() == CamFault::BUS_TIMEOUT);
}

// 5) READY 後無幀 M=20 → FRAME_TIMEOUT；SELFTEST 無幀也會
void testFrameTimeout() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  f.sim.setFrameEveryN(0xFFFFFFFFu);
  f.sim.setFrameAvailable(false);
  assert(runUntil(f.core, f.sim, 100u, CamState::FAULT, 128u));
  assert(f.core.fault() == CamFault::FRAME_TIMEOUT);
}

// 6) stop／reset → OFF，可重新握手；reset 清 frameCount
void testStopResetRehandshake() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  run(f.core, f.sim, 100u, 3u);
  assert(f.core.frameCount() >= 3u);

  f.core.stop();
  assert(f.core.state() == CamState::OFF);
  // stop 保留 frameCount
  assert(f.core.frameCount() >= 3u);

  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));

  f.core.reset();
  assert(f.core.state() == CamState::OFF);
  assert(f.core.fault() == CamFault::NONE);
  assert(f.core.frameCount() == 0u);

  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  assert(f.core.fault() == CamFault::NONE);
  assert(f.core.lastChipId() == 0x364C);
}

// 7) 初始化表 chunking：8／tick；參數可調
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

// 8) BUS_TIMEOUT 後 reset 復原
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

// 9) 參數 M 可調；1 Hz 不计幀逾時
void testParamFrameTimeoutAnd1Hz() {
  Fixture f;
  Ov3640Params p{};
  p.frameTimeoutTicks = 5;
  f.core.setParams(p);
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
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
  g.sim.setFrameEveryN(0xFFFFFFFFu);
  g.sim.setFrameAvailable(false);
  for (uint32_t i = 0; i < 10; ++i) {
    g.sim.advance();
    g.core.tick(1u);
  }
  assert(g.core.state() == CamState::READY);

  // 拒絕 1 kHz
  assert(g.core.tick(1000u) == false);
  assert(g.core.state() == CamState::READY);
}

// 10) 遙測不含影像：sink 有幀資料，CamStatus 沒有
void testTelemetryHasNoImage() {
  Fixture f;
  f.core.start();
  assert(runUntil(f.core, f.sim, 100u, CamState::READY, 64u));
  run(f.core, f.sim, 100u, 2u);
  assert(f.sink.frameEvents >= 2u);
  assert(f.sink.lastData[0] == 0xA0);  // sim 幀內容

  f.sink.pullTlm(f.core);
  const CamStatus s = f.sink.tlm;
  // 只檢查允許的欄位有值；無法從 status 讀回幀 bytes
  assert(s.frameCount == f.core.frameCount());
  assert(s.model == CamModel::OV3640);
  assert(s.role == CamRole::US_CAM);
  (void)s;
}

}  // namespace

int main() {
  testNormalPath();
  testBadChipId();
  testTypeRoleMismatch();
  testBusTimeout();
  testFrameTimeout();
  testStopResetRehandshake();
  testInitChunking();
  testRecoveryAfterBusFault();
  testParamFrameTimeoutAnd1Hz();
  testTelemetryHasNoImage();
  std::printf("Ov3640SimTest: all 10 tests passed\n");
  std::printf("FPP declares model=OV3640 role=US_CAM (see Ov3640Driver.fpp constants)\n");
  return 0;
}
