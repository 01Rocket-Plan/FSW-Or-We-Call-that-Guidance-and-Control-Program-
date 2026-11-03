// Esp32CamSimTest：host 端模擬測試（不需 F´）
// 驗證狀態機、逾時、指令、電傳欄位、I2C 時脈預設、預覽路徑隔離。
#include "Components/Camera/DsCam/Esp32CamCore.hpp"
#include "Components/Camera/DsCam/Esp32CamSimDevice.hpp"
#include "Components/Camera/DsCam/IGroundPreviewLink.hpp"
#include "Components/Camera/DsCam/ILocalSdStore.hpp"
#include <cstdio>

using namespace Components;

static int g_fail = 0;
#define CHECK(c)                                                                 \
  do {                                                                           \
    if (!(c)) {                                                                  \
      std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);                    \
      g_fail++;                                                                  \
    }                                                                            \
  } while (0)

namespace {

struct Recorder : public ICamSink {
  uint32_t faults = 0;
  CamFault lastFault = CamFault::NONE;
  CamState lastState = CamState::OFF;
  void onStateChange(CamState, CamState to) override { lastState = to; }
  void onFault(CamFault code, uint32_t) override {
    lastFault = code;
    faults++;
  }
};

struct Rig {
  Esp32CamSimDevice dev;
  Esp32CamCore core;
  Recorder rec;
  LocalSdStoreStub sd;
  GroundPreviewLinkStub preview;

  Rig(CamModel m = CamModel::ESP32_CAM, CamRole r = CamRole::DS_CAM) {
    core.configure(m, r, 3, 20);
    dev.configure(m, r);
    dev.setLocalSd(&sd);
    dev.setPreviewLink(&preview);
    core.setLink(&dev);
    core.setSink(&rec);
  }

  void step1Hz() { core.tick1Hz(); }
  void step100Hz() {
    dev.advance();
    core.tick100Hz();
  }

  bool driveToReady(int maxLoops = 200) {
    if (core.state() == CamState::OFF) {
      if (!core.powerOn()) {
        return false;
      }
    }
    for (int i = 0; i < maxLoops; ++i) {
      step1Hz();
      // 自檢期間多跑幾個 100 Hz tick，讓 FRAME 逾時與幀通知有機會發生
      for (int k = 0; k < 5; ++k) {
        step100Hz();
      }
      if (core.state() == CamState::READY) {
        uint8_t tmp[64];
        while (dev.recv(tmp, sizeof(tmp)) > 0) {
        }
        return true;
      }
      if (core.state() == CamState::FAULT) {
        return false;
      }
    }
    return false;
  }
};

}  // namespace

static void test1_happyPath() {
  std::printf("1) happy path READY + frameCount after CAPTURE_START\n");
  Rig r;
  CHECK(r.driveToReady());
  CHECK(r.core.state() == CamState::READY);
  CHECK(r.core.fault() == CamFault::NONE);
  uint32_t before = r.core.frameCount();
  CHECK(before == 0);
  // 未 CAPTURE_START：幀數不得增加
  for (int i = 0; i < 10; ++i) {
    r.step100Hz();
  }
  CHECK(r.core.frameCount() == before);
  CHECK(r.core.captureStart());
  for (int i = 0; i < 30; ++i) {
    r.step100Hz();
  }
  CHECK(r.core.frameCount() > before);
  CHECK(r.sd.frames() > 0);
  CHECK(r.preview.count() > 0);
}

static void test2_typeRoleMismatch() {
  std::printf("2) model mismatch -> TYPE_ROLE_MISMATCH\n");
  Rig r(CamModel::ESP32_CAM, CamRole::DS_CAM);
  r.dev.setFault(Esp32CamSimFault::WRONG_MODEL);
  CHECK(!r.driveToReady());
  CHECK(r.core.state() == CamState::FAULT);
  CHECK(r.core.fault() == CamFault::TYPE_ROLE_MISMATCH);

  Rig r2(CamModel::ESP32_CAM, CamRole::DS_CAM);
  r2.dev.setFault(Esp32CamSimFault::WRONG_ROLE);
  CHECK(!r2.driveToReady());
  CHECK(r2.core.state() == CamState::FAULT);
  CHECK(r2.core.fault() == CamFault::TYPE_ROLE_MISMATCH);
}

static void test3_busTimeout() {
  std::printf("3) link silent -> BUS_TIMEOUT\n");
  Rig r;
  r.dev.setFault(Esp32CamSimFault::SILENT);
  CHECK(!r.driveToReady());
  CHECK(r.core.state() == CamState::FAULT);
  CHECK(r.core.fault() == CamFault::BUS_TIMEOUT);
}

static void test4_frameTimeout() {
  std::printf("4) capture with NO_FRAMES -> FRAME_TIMEOUT\n");
  Rig r;
  CHECK(r.driveToReady());
  r.dev.setFault(Esp32CamSimFault::NO_FRAMES);
  CHECK(r.core.captureStart());
  for (int i = 0; i < 30; ++i) {
    r.step100Hz();
    if (r.core.state() == CamState::FAULT) {
      break;
    }
  }
  CHECK(r.core.state() == CamState::FAULT);
  CHECK(r.core.fault() == CamFault::FRAME_TIMEOUT);
}

static void test5_resetAndRehandshake() {
  std::printf("5) RESET -> OFF, frameCount=0, re-handshake\n");
  Rig r;
  CHECK(r.driveToReady());
  CHECK(r.core.captureStart());
  for (int i = 0; i < 30; ++i) {
    r.step100Hz();
  }
  CHECK(r.core.frameCount() > 0);
  CHECK(r.core.reset());
  CHECK(r.core.state() == CamState::OFF);
  CHECK(r.core.frameCount() == 0);
  CHECK(r.core.fault() == CamFault::NONE);
  CHECK(r.driveToReady());
  CHECK(r.core.state() == CamState::READY);
}

static void test6_captureRejectedWhenNotReady() {
  std::printf("6) CAPTURE_* rejected when not READY\n");
  Rig r;
  CHECK(!r.core.captureStart());
  CHECK(!r.core.captureStop());
  CHECK(r.core.powerOn());
  CHECK(!r.core.captureStart());
  r.step1Hz();
  CHECK(r.core.state() == CamState::SELFTEST);
  CHECK(!r.core.captureStart());
  CHECK(r.driveToReady());
  CHECK(r.core.captureStart());
  CHECK(r.core.captureStop());
}

static void test7_telemetryFourFields() {
  std::printf("7) CamStatus fields only (hsState/fault/frameCount/model+role)\n");
  Rig r;
  CHECK(r.driveToReady());
  CamStatus s = r.core.status();
  CHECK(s.hsState == CamState::READY);
  CHECK(s.fault == CamFault::NONE);
  CHECK(s.frameCount == 0);
  CHECK(s.model == CamModel::ESP32_CAM);
  CHECK(s.role == CamRole::DS_CAM);
  // 結構僅含上述欄位（無影像指標／緩衝）
  CHECK(sizeof(CamStatus) <= 16);
}

static void test8_i2cClockDefault() {
  std::printf("8) I2C clock default 400 kHz, tunable\n");
  Esp32CamCore core;
  CHECK(core.i2cClockHz() == 400000u);
  CHECK(core.i2cConfig().pinSda == CamI2cConfig::kPinSdaPlaceholder);
  CHECK(core.i2cConfig().pinScl == CamI2cConfig::kPinSclPlaceholder);
  core.setI2cClockHz(100000);
  CHECK(core.i2cClockHz() == 100000u);
  core.setI2cClockHz(0);  // 0 → 回到預設
  CHECK(core.i2cClockHz() == 400000u);
}

static void test9_previewPathIsolation() {
  std::printf("9) preview via IGroundPreviewLink only; ICamLink has no pixel bulk\n");
  Rig r;
  CHECK(r.driveToReady());
  CHECK(r.core.captureStart());
  for (int i = 0; i < 40; ++i) {
    r.step100Hz();
  }
  CHECK(r.core.frameCount() > 0);
  CHECK(r.preview.count() > 0);
  CHECK(r.preview.last().width == GroundPreviewSpec::kWidth);
  CHECK(r.preview.last().height == GroundPreviewSpec::kHeight);
  CHECK(r.sd.frames() > 0);
  // ICamLink 單次通知不得超過 header 大小（1+12）；無像素 bulk
  CHECK(r.dev.maxNotifyPayload() <= 13u);
}

int main() {
  test1_happyPath();
  test2_typeRoleMismatch();
  test3_busTimeout();
  test4_frameTimeout();
  test5_resetAndRehandshake();
  test6_captureRejectedWhenNotReady();
  test7_telemetryFourFields();
  test8_i2cClockDefault();
  test9_previewPathIsolation();

  if (g_fail != 0) {
    std::printf("Esp32CamSimTest: FAILED (%d checks)\n", g_fail);
    return 1;
  }
  std::printf("Esp32CamSimTest: 9/9 passed\n");
  return 0;
}
