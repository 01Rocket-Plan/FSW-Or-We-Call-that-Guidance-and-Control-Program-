#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../Components/EnvSensing/ImuDriver/imu_core.h"
#include "../Components/EnvSensing/UsMpu6050/mpu6050_core.h"

enum { REG_BANK_SEL=0x7F, BANK0_WHOAMI=0x00, BANK0_ACCEL=0x2D,
       WHO_20948=0xEA, WHO_20648=0xE0, MPU_WHO=0x75, MPU_ACCEL=0x3B,
       WHO_MPU=0x68 };
static const float ACCEL_SENS = 2048.0f;
static const float G0 = 9.80665f;

typedef enum { INJ_NORMAL=0, INJ_STUCK=1, INJ_GYROSAT=2, INJ_BUSERR=3,
               INJ_WHOAMI=4, INJ_ACCELSAT=5 } Inject;

typedef struct {
  Inject inject;
  uint8_t bank;
  int16_t accelRaw[3];
  int16_t gyroRaw[3];
  int busFailBudget;
  int is_mpu;
} FakeBus;

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
  fprintf(stderr, "FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); ++g_fail; } } while(0)

static bool fake_write(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
  FakeBus* b = (FakeBus*)ctx; (void)addr; (void)len;
  if (b->inject == INJ_BUSERR && b->busFailBudget > 0) { b->busFailBudget--; return false; }
  if (reg == REG_BANK_SEL && data) b->bank = (uint8_t)((data[0] >> 4) & 3);
  return true;
}
static void put16(uint8_t* d, int off, int16_t v) {
  d[off] = (uint8_t)((v >> 8) & 0xFF); d[off+1] = (uint8_t)(v & 0xFF);
}
static bool fake_read(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
  FakeBus* b = (FakeBus*)ctx; int i;
  int16_t a[3], g[3];
  if (b->inject == INJ_BUSERR && b->busFailBudget > 0) { b->busFailBudget--; return false; }
  memset(data, 0, len);
  if (!b->is_mpu && b->bank == 0 && reg == BANK0_WHOAMI && len >= 1) {
    data[0] = (addr == 0x69) ? (uint8_t)WHO_20948 : (uint8_t)WHO_20648;
    if (b->inject == INJ_WHOAMI) data[0] ^= 0xFF;
    return true;
  }
  if (b->is_mpu && reg == MPU_WHO && len >= 1) {
    data[0] = (b->inject == INJ_WHOAMI) ? (uint8_t)(WHO_MPU ^ 0xFF) : (uint8_t)WHO_MPU;
    return true;
  }
  a[0]=b->accelRaw[0]; a[1]=b->accelRaw[1]; a[2]=b->accelRaw[2];
  g[0]=b->gyroRaw[0]; g[1]=b->gyroRaw[1]; g[2]=b->gyroRaw[2];
  if (b->inject == INJ_GYROSAT) g[0] = 32000;
  if (b->inject == INJ_ACCELSAT) a[0] = 32000;
  if (!b->is_mpu && b->bank == 0 && reg == BANK0_ACCEL && len >= 12) {
    for (i=0;i<3;++i) put16(data, 2*i, a[i]);
    for (i=0;i<3;++i) put16(data, 6+2*i, g[i]);
    return true;
  }
  if (b->is_mpu && reg == MPU_ACCEL && len >= 14) {
    for (i=0;i<3;++i) put16(data, 2*i, a[i]);
    for (i=0;i<3;++i) put16(data, 8+2*i, g[i]);
    return true;
  }
  return true;
}

static void drive_ready(ImuCore* core, FakeBus* bus, double* t) {
  ImuSampleOut out; int i;
  bus->inject = INJ_NORMAL;
  for (i = 0; i < 5000; ++i) {
    *t += 0.001;
    imu_core_tick(core, *t, &out);
    if (imu_core_state(core) == IMU_DRV_READY) return;
  }
}

static void test_happy(void) {
  FakeBus bus = {0}; ImuBusOps ops; ImuCore* core; ImuParams p; ImuSampleOut out;
  double t = 0; uint32_t prev = 0; int i;
  bus.accelRaw[2] = (int16_t)ACCEL_SENS;
  ops.writeReg = fake_write; ops.readReg = fake_read; ops.ctx = &bus;
  core = imu_core_create(ops);
  p = imu_params_default(); p.selftestSamples = 5; p.stuckN = 20;
  imu_core_set_params(core, &p); imu_core_reset(core);
  drive_ready(core, &bus, &t);
  CHECK(imu_core_state(core) == IMU_DRV_READY, "reach READY");
  for (i = 0; i < 10; ++i) {
    bus.accelRaw[0] = (int16_t)i; t += 0.001;
    imu_core_tick(core, t, &out);
    CHECK(out.valid, "valid when READY");
    CHECK(out.seq == prev + 1, "seq increments");
    prev = out.seq;
    CHECK(fabsf(out.acc_mps2[2] - G0) < 0.5f, "Z accel ~1g");
  }
  imu_core_destroy(core);
}

static void test_whoami_failover(void) {
  FakeBus bus = {0}; ImuBusOps ops; ImuCore* core; ImuParams p; ImuSampleOut out;
  double t = 0; int i;
  bus.accelRaw[2] = (int16_t)ACCEL_SENS;
  ops.writeReg = fake_write; ops.readReg = fake_read; ops.ctx = &bus;
  core = imu_core_create(ops);
  p = imu_params_default(); p.selftestSamples = 3; p.whoamiFailThreshold = 2; p.busErrThreshold = 99;
  imu_core_set_params(core, &p); imu_core_reset(core);
  bus.inject = INJ_WHOAMI;
  for (i = 0; i < 200; ++i) { t += 0.001; imu_core_tick(core, t, &out);
    if (imu_core_active(core) == IMU_SENS_BACKUP) break; }
  CHECK(imu_core_active(core) == IMU_SENS_BACKUP, "failover to backup");
  bus.inject = INJ_NORMAL;
  for (i = 0; i < 5000; ++i) { t += 0.001; imu_core_tick(core, t, &out);
    if (imu_core_state(core) == IMU_DRV_READY) break; }
  CHECK(imu_core_state(core) == IMU_DRV_READY, "backup READY");
  CHECK(imu_core_failover(core) >= 1, "failover counted");
  imu_core_destroy(core);
}

static void test_stuck(void) {
  FakeBus bus = {0}; ImuBusOps ops; ImuCore* core; ImuParams p; ImuSampleOut out;
  double t = 0; int i;
  bus.accelRaw[2] = (int16_t)ACCEL_SENS;
  ops.writeReg = fake_write; ops.readReg = fake_read; ops.ctx = &bus;
  core = imu_core_create(ops);
  p = imu_params_default(); p.selftestSamples = 3; p.stuckN = 5; p.busErrThreshold = 99;
  imu_core_set_params(core, &p); imu_core_reset(core);
  drive_ready(core, &bus, &t);
  bus.inject = INJ_STUCK;
  for (i = 0; i < 50; ++i) { t += 0.001; imu_core_tick(core, t, &out); }
  CHECK(imu_core_stuck(core) >= 1 || imu_core_failover(core) >= 1, "stuck/failover");
  imu_core_destroy(core);
}

static void test_gyro_sat(void) {
  FakeBus bus = {0}; ImuBusOps ops; ImuCore* core; ImuParams p; ImuSampleOut out;
  double t = 0;
  bus.accelRaw[2] = (int16_t)ACCEL_SENS;
  ops.writeReg = fake_write; ops.readReg = fake_read; ops.ctx = &bus;
  core = imu_core_create(ops);
  p = imu_params_default(); p.selftestSamples = 3;
  imu_core_set_params(core, &p); imu_core_reset(core);
  drive_ready(core, &bus, &t);
  bus.inject = INJ_GYROSAT; bus.accelRaw[0] = 1; t += 0.001;
  imu_core_tick(core, t, &out);
  CHECK(out.valid && out.gyroSat, "gyroSat");
  imu_core_destroy(core);
}

static void test_no_valid_early(void) {
  FakeBus bus = {0}; ImuBusOps ops; ImuCore* core; ImuParams p; ImuSampleOut out;
  double t = 0; int i;
  bus.accelRaw[2] = (int16_t)ACCEL_SENS;
  ops.writeReg = fake_write; ops.readReg = fake_read; ops.ctx = &bus;
  core = imu_core_create(ops);
  p = imu_params_default(); p.selftestSamples = 100;
  imu_core_set_params(core, &p); imu_core_reset(core);
  for (i = 0; i < 30; ++i) { t += 0.001; imu_core_tick(core, t, &out); CHECK(!out.valid, "invalid before READY"); }
  imu_core_destroy(core);
}

static void test_mpu(void) {
  FakeBus bus = {0}; ImuBusOps ops; Mpu6050Core* core; MpuParams p; MpuSampleOut out;
  double t = 0; int i;
  bus.is_mpu = 1; bus.accelRaw[2] = (int16_t)ACCEL_SENS;
  ops.writeReg = fake_write; ops.readReg = fake_read; ops.ctx = &bus;
  core = mpu_core_create(ops);
  p = mpu_params_default(); p.selftestSamples = 3;
  p.accelSatThresh_mps2 = 100.0f; p.gyroSatThresh_rads = 10.0f;
  mpu_core_set_params(core, &p); mpu_core_reset(core);
  for (i = 0; i < 5000; ++i) { t += 0.001; mpu_core_tick(core, t, &out);
    if (mpu_core_state(core) == IMU_DRV_READY) break; }
  CHECK(mpu_core_state(core) == IMU_DRV_READY, "MPU READY");
  bus.inject = INJ_ACCELSAT; bus.gyroRaw[0] = 1; t += 0.001; mpu_core_tick(core, t, &out);
  CHECK(out.valid && out.accelSat, "MPU accelSat");
  bus.inject = INJ_GYROSAT; bus.accelRaw[0] = 2; t += 0.001; mpu_core_tick(core, t, &out);
  CHECK(out.valid && out.gyroSat, "MPU gyroSat");
  mpu_core_destroy(core);
}

static void test_mpu_whoami(void) {
  FakeBus bus = {0}; ImuBusOps ops; Mpu6050Core* core; MpuParams p; MpuSampleOut out;
  double t = 0; int i;
  bus.is_mpu = 1;
  ops.writeReg = fake_write; ops.readReg = fake_read; ops.ctx = &bus;
  core = mpu_core_create(ops);
  p = mpu_params_default(); p.whoamiFailThreshold = 2;
  mpu_core_set_params(core, &p); mpu_core_reset(core);
  bus.inject = INJ_WHOAMI;
  for (i = 0; i < 100; ++i) { t += 0.001; mpu_core_tick(core, t, &out); }
  CHECK(mpu_core_state(core) == IMU_DRV_FAULT, "MPU WHOAMI FAULT");
  CHECK(mpu_core_fault(core) == IMU_FAULT_WHOAMI, "reason WhoAmI");
  mpu_core_destroy(core);
}

int main(void) {
  test_happy();
  test_whoami_failover();
  test_stuck();
  test_gyro_sat();
  test_no_valid_early();
  test_mpu();
  test_mpu_whoami();
  if (g_fail) { fprintf(stderr, "%d check(s) failed\n", g_fail); return 1; }
  puts("ALL_HOST_TESTS_PASSED");
  return 0;
}
