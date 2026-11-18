// CamTimingTest：下級相機 DsCam（ESP32-CAM 模組內建 OV2640；上級 UsCam 是 OV7670，不在此測）時序模型。期望值是手算（見註解），出處見 CamTiming.hpp。
#include "Components/Camera/DsCam/CamTiming.hpp"
#include "Components/Camera/DsCam/Esp32CamCore.hpp"
#include "Components/Camera/DsCam/Esp32CamSimDevice.hpp"
#include <gtest/gtest.h>

using namespace Components;
using namespace Components::CamTiming;

TEST(CamTiming, DocumentedDefaults) {
  EXPECT_EQ(kXclkHzDefault, 20000000u);   // esp32-camera README xclk_freq_hz
  EXPECT_EQ(kJpegQualityDefault, 12u);    // esp32-camera README jpeg_quality
  EXPECT_EQ(kJpegQualityMax, 63u);        // esp_camera.h "0-63"
  EXPECT_EQ(kSccbClockHzDefault, 100000u);  // esp32-camera Kconfig SCCB_CLK_FREQ
  EXPECT_LE(kSccbClockHzDefault, kOv2640SccbClockHzMax);  // OV2640 Table 9 ≤ 400 kHz
  EXPECT_EQ(kOv2640SvgaMaxFps, 30u);      // OV2640 datasheet p.1
  EXPECT_TRUE(jpegQualityValid(kJpegQualityDefault));
  EXPECT_TRUE(jpegQualityValid(0));
  EXPECT_FALSE(jpegQualityValid(64));
  // 和 repo 既有的 ICamLink I2C 預設一致
  EXPECT_EQ(kCamLinkI2cClockHz, +Esp32CamCore::kDefaultI2cClockHz);  // C++14：+ 避免 ODR-use
}

TEST(CamTiming, I2cTransactionTime) {
  // 幀通知 13 B：(1 + 13) × 9 = 126 clk；@400 kHz = 315 000 ns
  EXPECT_EQ(i2cTransactionClocks(Esp32CamSimDevice::kFrameNotifyBytes), 126u);
  EXPECT_EQ(i2cTransactionTimeNs(Esp32CamSimDevice::kFrameNotifyBytes, 400000), 315000u);
  // CamLinkPacket.md 最大幀 68 B：(1 + 68) × 9 = 621 clk；@400 kHz = 1 552 500 ns；@100 kHz = 6 210 000 ns
  EXPECT_EQ(i2cTransactionTimeNs(68, 400000), 1552500u);
  EXPECT_EQ(i2cTransactionTimeNs(68, 100000), 6210000u);
  EXPECT_EQ(i2cTransactionTimeNs(68, 0), 0u);
  // 一個 100 Hz tick（10 ms）內，最大幀的 I2C 時間只佔 15.5 %
  EXPECT_LT(i2cTransactionTimeNs(68, kCamLinkI2cClockHz), 10000000u);
}

TEST(CamTiming, SccbRegisterWriteTime) {
  // 3 phase × 9 = 27 clk；@100 kHz = 270 000 ns；@400 kHz = 67 500 ns
  EXPECT_EQ(sccbRegWriteClocks(), 27u);
  EXPECT_EQ(sccbRegWriteTimeNs(1, kSccbClockHzDefault), 270000u);
  EXPECT_EQ(sccbRegWriteTimeNs(1, kOv2640SccbClockHzMax), 67500u);
  EXPECT_EQ(sccbRegWriteTimeNs(100, kSccbClockHzDefault), 27000000u);  // 100 個暫存器 27 ms
}

TEST(CamTiming, SensorFramePeriodAndSimCadence) {
  // 30 fps → 33 333 333 ns
  EXPECT_EQ(sensorMinFramePeriodNs(kOv2640SvgaMaxFps), 33333333u);
  EXPECT_EQ(sensorMinFramePeriodNs(0), 0u);
  // Sim：5 tick × 10 ms = 50 ms ≥ 33.3 ms（不會比感測器快）
  const uint64_t simPeriodNs = static_cast<uint64_t>(Esp32CamSimDevice::kSimFrameIntervalTicks) *
                               Esp32CamSimDevice::kSimTickUs * 1000u;
  EXPECT_EQ(simPeriodNs, 50000000u);
  EXPECT_GE(simPeriodNs, sensorMinFramePeriodNs(kOv2640SvgaMaxFps));
}

TEST(CamTiming, SimEmitsFramesAtDocumentedCadence) {
  Esp32CamSimDevice dev;
  dev.configure(CamModel::ESP32_CAM, CamRole::DS_CAM);
  const uint8_t start = 0xB1;  // kCmdCapStart（Esp32CamSimDevice.cpp）
  dev.send(&start, 1);
  for (uint32_t i = 0; i < 100; ++i) dev.advance();  // 1 s @100 Hz
  EXPECT_EQ(dev.framesPushed(), 100u / Esp32CamSimDevice::kSimFrameIntervalTicks);  // 20 幀
  EXPECT_LE(dev.framesPushed(), kOv2640SvgaMaxFps);
  EXPECT_LE(dev.maxNotifyPayload(), +Esp32CamSimDevice::kFrameNotifyBytes);
}
