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
    if (drv.m_lastTlm.get_camState() == CamState::READY) return;
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
  assert(drv.m_lastTlm.get_camState() == CamState::READY);
  assert(drv.m_lastTlm.get_fault() == CamFault::NONE);
  assert(drv.m_lastTlm.get_model() == CamModel::OV3640);
  assert(drv.m_lastTlm.get_role() == CamRole::US_CAM);
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
  assert(drv.m_endChunks >= 1);

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
    if (drv.m_lastTlm.get_camState() == CamState::READY) break;
  }
  assert(drv.m_lastTlm.get_camState() == CamState::READY);
  assert(drv.m_sccbWrites >= 12);
  assert(drv.m_sccbReads >= 2);
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
  assert(drv.m_lastTlm.get_camState() == CamState::FAULT);
  assert(drv.m_lastTlm.get_fault() == CamFault::TYPE_ROLE_MISMATCH ||
         drv.m_lastFault == CamFault::TYPE_ROLE_MISMATCH);
}

// Binding #2: stream 2KB chunks; frame# + chunkIndex; last has END
static void testChunkStreaming() {
  Ov3640SimDevice frames;
  frames.setReportedFrameLength(5000);  // → 3 chunks: 2048+2048+904
  Ov3640Driver drv("ov");
  drv.hostSetParams(3, 20, 8, CamModel::OV3640, CamRole::US_CAM, 0x3C, 5, 3, 65536);
  drv.hostLoadParams();
  drv.configure(&frames);
  drv.hostSetSccbConnected(true);
  drv.hostSetImageConnected(true);
  drv.hostCmdPowerOn();
  for (int i = 0; i < 64; ++i) {
    frames.advance();
    drv.hostSched100Hz();
    if (drv.m_lastTlm.get_camState() == CamState::READY) break;
  }
  assert(drv.m_lastTlm.get_camState() == CamState::READY);
  drv.hostCmdCaptureStart();
  const U32 outsBefore = drv.m_imageOuts;
  const U32 endsBefore = drv.m_endChunks;
  frames.advance();
  drv.hostSched100Hz();
  assert(drv.m_imageOuts == outsBefore + 3);
  assert(drv.m_endChunks == endsBefore + 1);
  assert(drv.m_lastChunkIndex == 2);
  assert(drv.m_lastFlags & static_cast<U8>(CamChunkFlag::END));
  assert(!(drv.m_lastFlags & static_cast<U8>(CamChunkFlag::TRUNCATED)));
  assert(drv.m_lastFrameNumber == 1);
  // Stop capture before 1 Hz tlm refresh (1 Hz would otherwise poll another frame).
  drv.hostCmdCaptureStop();
  drv.hostSched1Hz();
  assert(drv.m_lastTlm.get_frameCount() == 1);
}

// Binding #3: FRAME_MAX_BYTES exceeded → last chunk TRUNCATED, FrameTruncated event, no frameCount++
static void testFrameMaxBytesTruncation() {
  Ov3640SimDevice frames;
  frames.setReportedFrameLength(5000);
  Ov3640Driver drv("ov");
  // Cap at 3000 bytes → chunks 2048+952 with TRUNCATED|END; do not count frame
  drv.hostSetParams(3, 20, 8, CamModel::OV3640, CamRole::US_CAM, 0x3C, 5, 3, 3000);
  drv.hostLoadParams();
  drv.configure(&frames);
  drv.hostSetSccbConnected(true);
  drv.hostSetImageConnected(true);
  drv.hostCmdPowerOn();
  for (int i = 0; i < 64; ++i) {
    frames.advance();
    drv.hostSched100Hz();
    if (drv.m_lastTlm.get_camState() == CamState::READY) break;
  }
  assert(drv.m_lastTlm.get_camState() == CamState::READY);
  drv.hostCmdCaptureStart();
  const U32 truncBefore = drv.m_truncEvents;
  const U32 outsBefore = drv.m_imageOuts;
  frames.advance();
  drv.hostSched100Hz();
  assert(drv.m_truncEvents == truncBefore + 1);
  assert(drv.m_lastTruncFrame == 1);
  assert(drv.m_lastTruncBytes == 3000);
  assert(drv.m_lastFlags & static_cast<U8>(CamChunkFlag::END));
  assert(drv.m_lastFlags & static_cast<U8>(CamChunkFlag::TRUNCATED));
  assert(drv.m_imageOuts == outsBefore + 2);  // 2048 + 952
  drv.hostCmdCaptureStop();
  drv.hostSched1Hz();
  assert(drv.m_lastTlm.get_frameCount() == 0);      // truncated frames do not count
}

// Binding #1 via adapter: RecoverAttempt on 1 Hz
static void testAdapterFaultRecoverEvent() {
  Ov3640DriverSim drv("ovSim");
  drv.hostSetParams(3, 20, 8, CamModel::OV3640, CamRole::US_CAM, 0x3C, 5, 3, 65536);
  drv.hostLoadParams();
  drv.hostCmdPowerOn();
  for (int i = 0; i < 64; ++i) drv.hostSched100Hz();
  assert(drv.m_lastTlm.get_camState() == CamState::READY);
  drv.hostCmdSimInject(static_cast<U8>(Ov3640SimFault::BUS_DOWN));
  // drain into FAULT during init of next... actually need fault during capture or re-init
  // Inject bus down then RESET+POWER to hit INIT with bus down
  drv.hostCmdReset();
  drv.hostCmdPowerOn();
  for (int i = 0; i < 32; ++i) drv.hostSched100Hz();
  assert(drv.m_lastTlm.get_camState() == CamState::FAULT);
  const U32 before = drv.m_recoverEvents;
  for (int i = 0; i < 5; ++i) drv.hostSched1Hz();
  assert(drv.m_recoverEvents == before + 1);
  assert(drv.m_lastRecoverAttempt == 1);
  assert(drv.m_lastRecoverLimit == 3);
}

int main() {
  testCaptureRejectedNotReady();
  testTelemetryCamStatusOnly();
  testImagePortRouting();
  testFlightAdapterWithInjectedFrames();
  testTypeRoleMismatchViaParams();
  testChunkStreaming();
  testFrameMaxBytesTruncation();
  testAdapterFaultRecoverEvent();
  std::printf("Ov3640AdapterTest: all 8 tests passed\n");
  return 0;
}
