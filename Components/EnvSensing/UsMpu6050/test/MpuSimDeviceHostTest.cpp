// MpuSimDevice setTruth host 測試（不需 F´）：見 ../../run_host_tests.sh
#include "Components/EnvSensing/UsMpu6050/MpuSimDevice.hpp"
#include "Components/EnvSensing/UsMpu6050/mpu6050_core.h"
#include <cmath>
#include <cstdio>
#include <cstdint>

using namespace Components;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

static const float GYRO_SENS_2000_DPS = 16.4f;
static const float DEG2RAD = 0.017453292519943295f;
static const float G0_MPS2 = 9.80665f;

static void unpack14(const uint8_t* data, int16_t a[3], int16_t g[3]) {
  for (int i = 0; i < 3; ++i) {
    a[i] = static_cast<int16_t>((data[2 * i] << 8) | data[2 * i + 1]);
    g[i] = static_cast<int16_t>((data[8 + 2 * i] << 8) | data[8 + 2 * i + 1]);
  }
}

static void testDefaultReset() {
  MpuSimDevice dev;
  int16_t a[3] = {-1, -1, -1};
  int16_t g[3] = {-1, -1, -1};
  dev.getTruth(a, g);
  CHECK(a[0] == 0 && a[1] == 0 && a[2] == 2048);
  CHECK(g[0] == 0 && g[1] == 0 && g[2] == 0);

  uint8_t buf[14];
  CHECK(dev.readReg(0x68, 0x3B, buf, 14));
  int16_t ar[3], gr[3];
  unpack14(buf, ar, gr);
  CHECK(ar[0] == 0 && ar[1] == 0 && ar[2] == 2048);
  CHECK(gr[0] == 0 && gr[1] == 0 && gr[2] == 0);

  uint8_t who = 0;
  CHECK(dev.readReg(0x68, 0x75, &who, 1) && who == 0x68);
}

static void testSetTruthRegister() {
  MpuSimDevice dev;
  const int16_t accel[3] = {100, -200, 3072};
  const int16_t gyro[3] = {16, -32, 1640};
  dev.setTruth(accel, gyro);

  int16_t a[3], g[3];
  dev.getTruth(a, g);
  CHECK(a[0] == 100 && a[1] == -200 && a[2] == 3072);
  CHECK(g[0] == 16 && g[1] == -32 && g[2] == 1640);

  uint8_t buf[14];
  CHECK(dev.readReg(0x68, 0x3B, buf, 14));
  int16_t ar[3], gr[3];
  unpack14(buf, ar, gr);
  CHECK(ar[0] == 100 && ar[1] == -200 && ar[2] == 3072);
  CHECK(gr[0] == 16 && gr[1] == -32 && gr[2] == 1640);

  // setTruth 後 reset 要回到預設靜置
  dev.reset();
  dev.getTruth(a, g);
  CHECK(a[0] == 0 && a[1] == 0 && a[2] == 2048);
  CHECK(g[0] == 0 && g[1] == 0 && g[2] == 0);
}

static void testSetTruthThroughCore() {
  MpuSimDevice dev;
  const int16_t accel[3] = {0, 0, 2048};      // 1 g
  const int16_t gyro[3] = {1640, 0, 0};       // 100 deg/s at 16.4 LSB/(deg/s)
  dev.setTruth(accel, gyro);

  Mpu6050Core* core = mpu_core_create(dev.busOps());
  CHECK(core != nullptr);
  // 縮短自測：預設 10 筆，再加上 INIT 的等待 tick，遠小於 200 tick
  MpuParams p = mpu_params_default();
  p.selftestSamples = 3;
  p.stuckN = 1000;  // 本測試要靜置比較，避免 stuck 誤判
  mpu_core_set_params(core, &p);
  mpu_core_reset(core);

  MpuSampleOut last{};
  int i = 0;
  for (; i < 200; ++i) {
    MpuSampleOut out{};
    mpu_core_tick(core, i * 0.01, &out);
    if (mpu_core_state(core) == IMU_DRV_READY) {
      break;
    }
  }
  CHECK(mpu_core_state(core) == IMU_DRV_READY);
  // 再 tick 一拍才會從 READY 路徑吐出有效樣本
  CHECK(mpu_core_tick(core, i * 0.01 + 0.01, &last));
  CHECK(last.valid);
  CHECK(std::fabs(last.accel_sensor_mps2[2] - G0_MPS2) < 0.05f);
  CHECK(std::fabs(last.accel_sensor_mps2[0]) < 0.05f);
  CHECK(std::fabs(last.accel_sensor_mps2[1]) < 0.05f);
  const float expect_gyro = (1640.0f / GYRO_SENS_2000_DPS) * DEG2RAD;
  CHECK(std::fabs(last.gyro_sensor_rads[0] - expect_gyro) < 0.005f);
  CHECK(std::fabs(last.gyro_sensor_rads[1]) < 0.001f);
  CHECK(std::fabs(last.gyro_sensor_rads[2]) < 0.001f);

  // 運轉中改真值，下一筆有效樣本要跟上
  const int16_t accel2[3] = {2048, 0, 0};
  const int16_t gyro2[3] = {0, 0, 0};
  dev.setTruth(accel2, gyro2);
  MpuSampleOut out{};
  CHECK(mpu_core_tick(core, 0.90, &out));
  CHECK(out.valid);
  CHECK(std::fabs(out.accel_sensor_mps2[0] - G0_MPS2) < 0.05f);
  CHECK(std::fabs(out.accel_sensor_mps2[2]) < 0.05f);
  CHECK(std::fabs(out.gyro_sensor_rads[0]) < 0.001f);
  mpu_core_destroy(core);
}

static void testNullSetTruthIgnored() {
  MpuSimDevice dev;
  const int16_t accel[3] = {1, 2, 3};
  const int16_t gyro[3] = {4, 5, 6};
  dev.setTruth(accel, gyro);
  dev.setTruth(nullptr, gyro);
  dev.setTruth(accel, nullptr);
  int16_t a[3], g[3];
  dev.getTruth(a, g);
  CHECK(a[0] == 1 && a[1] == 2 && a[2] == 3);
  CHECK(g[0] == 4 && g[1] == 5 && g[2] == 6);
}

int main() {
  testDefaultReset();
  testSetTruthRegister();
  testSetTruthThroughCore();
  testNullSetTruthIgnored();
  if (g_fail) {
    std::printf("MpuSimDeviceHostTest FAILED (%d)\n", g_fail);
    return 1;
  }
  std::printf("MpuSimDeviceHostTest OK\n");
  return 0;
}
