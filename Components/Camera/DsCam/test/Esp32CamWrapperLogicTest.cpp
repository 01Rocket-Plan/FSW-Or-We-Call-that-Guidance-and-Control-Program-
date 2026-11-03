// Esp32CamWrapperLogicTest：指令結果對應（不需 F´ Ac）
#include "Components/Camera/DsCam/Esp32CamCmd.hpp"
#include "Components/Camera/DsCam/Esp32CamCore.hpp"
#include "Components/Camera/DsCam/Esp32CamSimDevice.hpp"
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

int main() {
  std::printf("W1) cmd result mapping\n");
  CHECK(esp32CamCmdResult(true) == Esp32CamCmdResult::OK);
  CHECK(esp32CamCmdResult(false) == Esp32CamCmdResult::EXECUTION_ERROR);

  std::printf("W2) CAPTURE_* -> EXECUTION_ERROR when not READY\n");
  Esp32CamSimDevice dev;
  Esp32CamCore core;
  core.configure(CamModel::ESP32_CAM, CamRole::DS_CAM, 3, 20);
  dev.configure(CamModel::ESP32_CAM, CamRole::DS_CAM);
  core.setLink(&dev);
  CHECK(esp32CamCmdResult(core.captureStart()) == Esp32CamCmdResult::EXECUTION_ERROR);
  CHECK(esp32CamCmdResult(core.captureStop()) == Esp32CamCmdResult::EXECUTION_ERROR);
  CHECK(esp32CamCmdResult(core.powerOn()) == Esp32CamCmdResult::OK);
  CHECK(esp32CamCmdResult(core.captureStart()) == Esp32CamCmdResult::EXECUTION_ERROR);

  std::printf("W3) defaults N/M/I2C clock + placeholder pins\n");
  CHECK(core.i2cClockHz() == 400000u);
  CHECK(Esp32CamCore::kDefaultMissLimit == 3);
  CHECK(Esp32CamCore::kDefaultFrameLimit == 20);
  CHECK(CamI2cConfig::kPinSdaPlaceholder == 0xFF);
  CHECK(CamI2cConfig::kPinSclPlaceholder == 0xFF);

  std::printf("W4) CamStatus.hsState field (not reserved 'state')\n");
  CamStatus s = core.status();
  CHECK(s.hsState == CamState::INIT);

  if (g_fail) {
    std::printf("Esp32CamWrapperLogicTest: FAILED (%d)\n", g_fail);
    return 1;
  }
  std::printf("Esp32CamWrapperLogicTest: passed\n");
  return 0;
}
