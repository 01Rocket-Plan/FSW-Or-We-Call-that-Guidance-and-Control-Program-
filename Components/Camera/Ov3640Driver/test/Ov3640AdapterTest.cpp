// Ov3640AdapterTest：F´ 外殼 host 測試（用 test/host_ac 取代 ComponentAc）
#include "Components/Camera/Ov3640Driver/Ov3640Driver.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640DriverSim.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640SimDevice.hpp"

#include <cassert>
#include <cstdio>

using namespace Components;

static void runSimToReady(Ov3640DriverSim& drv, int maxTicks = 64) {
  drv.hostCmdPowerOn();
  for (int i = 0; i < maxTicks; ++i) {
    drv.hostSched100Hz();
    if (drv.m_lastTlm.camState == CamState::READY) return;
  }
}

static void testCaptureRejectedNotReady() {
  Ov3640DriverSim drv("ovSim");
  drv.hostLoadParams();
  drv.hostCmdCaptureStart();
  assert(drv.m_lastResp == Fw::CmdResponse::EXECUTION_ERROR);
  assert(drv.m_cmdResponses >= 1);
}

static void testTelemetryCamStatusOnly() {
  Ov3640DriverSim drv("ovSim");
  drv.hostLoadParams();
  runSimToReady(drv);
  assert(drv.m_lastTlm.camState == CamState::READY);
  assert(drv.m_lastTlm.fault == CamFault::NONE);
  assert(drv.m_lastTlm.model == CamModel::OV3640);
  assert(drv.m_lastTlm.role == CamRole::US_CAM);
  // Status has no image payload fields
  assert(drv.m_tlmWrites >= 1);
}

static void testImagePortRouting() {
  Ov3640DriverSim drv("ovSim");
  drv.hostLoadParams();
  drv.hostSetImageConnected(true);
  runSimToReady(drv);
  drv.hostCmdCaptureStart();
  assert(drv.m_lastResp == Fw::CmdResponse::OK);
  const U32 before = drv.m_imageOuts;
  for (int i = 0; i < 5; ++i) drv.hostSched100Hz();
  assert(drv.m_imageOuts > before);
  assert(drv.m_lastImageSize > 0);

  // Disconnect image port: frames should not route
  const U32 mid = drv.m_imageOuts;
  drv.hostSetImageConnected(false);
  for (int i = 0; i < 5; ++i) drv.hostSched100Hz();
  assert(drv.m_imageOuts == mid);
}

static void testFlightAdapterWithInjectedFrames() {
  Ov3640SimDevice frames;
  Ov3640Driver drv("ov");
  drv.hostLoadParams();
  drv.configure(&frames);
  drv.hostSetSccbConnected(true);
  drv.hostSetImageConnected(true);
  drv.hostCmdPowerOn();
  for (int i = 0; i < 64; ++i) {
    frames.advance();
    drv.hostSched100Hz();
    if (drv.m_lastTlm.camState == CamState::READY) break;
  }
  assert(drv.m_lastTlm.camState == CamState::READY);
  assert(drv.m_sccbWrites >= 12);  // init table
  assert(drv.m_sccbReads >= 2);    // chip id
  drv.hostCmdCaptureStart();
  assert(drv.m_lastResp == Fw::CmdResponse::OK);
  const U32 before = drv.m_imageOuts;
  for (int i = 0; i < 3; ++i) {
    frames.advance();
    drv.hostSched100Hz();
  }
  assert(drv.m_imageOuts > before);
}

static void testTypeRoleMismatchViaParams() {
  Ov3640DriverSim drv("ovSim");
  drv.hostSetParams(3, 20, 8, CamModel::ESP32_CAM, CamRole::US_CAM, 0x3C);
  drv.hostLoadParams();
  drv.hostCmdPowerOn();
  for (int i = 0; i < 64; ++i) drv.hostSched100Hz();
  assert(drv.m_lastTlm.camState == CamState::FAULT);
  assert(drv.m_lastTlm.fault == CamFault::TYPE_ROLE_MISMATCH ||
         drv.m_lastFault == CamFault::TYPE_ROLE_MISMATCH);
}

int main() {
  testCaptureRejectedNotReady();
  testTelemetryCamStatusOnly();
  testImagePortRouting();
  testFlightAdapterWithInjectedFrames();
  testTypeRoleMismatchViaParams();
  std::printf("Ov3640AdapterTest: all 5 tests passed\n");
  return 0;
}
