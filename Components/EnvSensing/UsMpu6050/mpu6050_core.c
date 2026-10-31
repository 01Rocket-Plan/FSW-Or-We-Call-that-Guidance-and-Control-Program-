#include "mpu6050_core.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

enum {
  MPU_SMPLRT_DIV = 0x19,
  MPU_CONFIG = 0x1A,
  MPU_GYRO_CONFIG = 0x1B,
  MPU_ACCEL_CONFIG = 0x1C,
  MPU_ACCEL_XOUT_H = 0x3B,
  MPU_PWR_MGMT_1 = 0x6B,
  MPU_WHO_AM_I = 0x75,
  WHOAMI_MPU6050 = 0x68,
  MPU_PWR1_RESET = 0x80,
  MPU_PWR1_WAKE_PLL = 0x01,
  MPU_GYRO_FS_2000 = 0x18,
  MPU_ACCEL_FS_16G = 0x18
};
static const float GYRO_SENS_2000_DPS = 16.4f;
static const float ACCEL_SENS_16G = 2048.0f;
static const float DEG2RAD = 0.017453292519943295f;
static const float G0_MPS2 = 9.80665f;

struct Mpu6050Core {
  ImuBusOps bus;
  MpuParams params;
  ImuDrvState state;
  ImuFaultReason faultReason;
  uint8_t initStep;
  uint32_t waitTicks;
  uint32_t selftestCount;
  uint32_t whoamiFails;
  uint32_t busErrCount;
  uint32_t stuckCount;
  uint32_t stuckRun;
  uint32_t seq;
  float lastA[3];
  float lastG[3];
  int haveLast;
};

MpuParams mpu_params_default(void) {
  MpuParams p;
  memset(&p, 0, sizeof(p));
  p.stuckN = 50; p.busErrThreshold = 5; p.whoamiFailThreshold = 3;
  p.selftestSamples = 10; p.gyroSatThresh_rads = 34.0f; p.accelSatThresh_mps2 = 156.0f;
  p.i2cAddr = 0x68;
  return p;
}

Mpu6050Core* mpu_core_create(ImuBusOps bus) {
  Mpu6050Core* c = (Mpu6050Core*)calloc(1, sizeof(Mpu6050Core));
  if (!c) return NULL;
  c->bus = bus; c->params = mpu_params_default(); c->state = IMU_DRV_OFF;
  return c;
}
void mpu_core_destroy(Mpu6050Core* c) { free(c); }
void mpu_core_set_params(Mpu6050Core* c, const MpuParams* p) { if (c && p) c->params = *p; }
void mpu_core_reset(Mpu6050Core* c) {
  if (!c) return;
  c->state = IMU_DRV_OFF; c->initStep = 0; c->waitTicks = 0; c->selftestCount = 0;
  c->whoamiFails = 0; c->stuckRun = 0; c->faultReason = IMU_FAULT_NONE; c->haveLast = 0;
}
ImuDrvState mpu_core_state(const Mpu6050Core* c) { return c->state; }
ImuFaultReason mpu_core_fault(const Mpu6050Core* c) { return c->faultReason; }
uint32_t mpu_core_seq(const Mpu6050Core* c) { return c->seq; }

static int write_u8(Mpu6050Core* c, uint8_t reg, uint8_t val) {
  return c->bus.writeReg(c->bus.ctx, c->params.i2cAddr, reg, &val, 1);
}
static int read_u8(Mpu6050Core* c, uint8_t reg, uint8_t* val) {
  return c->bus.readReg(c->bus.ctx, c->params.i2cAddr, reg, val, 1);
}
static int read_raw(Mpu6050Core* c, int16_t accel[3], int16_t gyro[3]) {
  uint8_t buf[14]; int i;
  if (!c->bus.readReg(c->bus.ctx, c->params.i2cAddr, (uint8_t)MPU_ACCEL_XOUT_H, buf, 14)) return 0;
  for (i = 0; i < 3; ++i) {
    accel[i] = (int16_t)((buf[2*i] << 8) | buf[2*i+1]);
    gyro[i]  = (int16_t)((buf[8+2*i] << 8) | buf[8+2*i+1]);
  }
  return 1;
}
static void convert(const int16_t accel[3], const int16_t gyro[3], float acc[3], float gyr[3]) {
  int i;
  for (i = 0; i < 3; ++i) {
    acc[i] = (accel[i] / ACCEL_SENS_16G) * G0_MPS2;
    gyr[i] = (gyro[i] / GYRO_SENS_2000_DPS) * DEG2RAD;
  }
}
static void enter_fault(Mpu6050Core* c, ImuFaultReason r) {
  c->state = IMU_DRV_FAULT; c->faultReason = r;
}
static void note_bus_fail(Mpu6050Core* c) {
  c->busErrCount++;
  if (c->busErrCount >= c->params.busErrThreshold) enter_fault(c, IMU_FAULT_BUS);
}
static void emit_invalid(Mpu6050Core* c, MpuSampleOut* out, double t_s) {
  memset(out, 0, sizeof(*out));
  out->t_s = t_s; out->seq = c->seq; out->valid = false;
}

static int step_init(Mpu6050Core* c) {
  uint8_t id;
  switch (c->initStep) {
    case 0:
      if (!write_u8(c, (uint8_t)MPU_PWR_MGMT_1, (uint8_t)MPU_PWR1_RESET)) { note_bus_fail(c); return 0; }
      c->waitTicks = 10; c->initStep = 1; return 1;
    case 1:
      if (c->waitTicks > 0) { c->waitTicks--; return 1; }
      c->initStep = 2; return 1;
    case 2:
      if (!read_u8(c, (uint8_t)MPU_WHO_AM_I, &id)) { note_bus_fail(c); return 0; }
      if (id != (uint8_t)WHOAMI_MPU6050) {
        c->whoamiFails++;
        if (c->whoamiFails >= c->params.whoamiFailThreshold) enter_fault(c, IMU_FAULT_WHOAMI);
        return 0;
      }
      c->initStep = 3; return 1;
    case 3:
      if (!write_u8(c, (uint8_t)MPU_PWR_MGMT_1, (uint8_t)MPU_PWR1_WAKE_PLL)) { note_bus_fail(c); return 0; }
      c->initStep = 4; return 1;
    case 4:
      if (!write_u8(c, (uint8_t)MPU_SMPLRT_DIV, 0x00)) { note_bus_fail(c); return 0; }
      c->initStep = 5; return 1;
    case 5:
      if (!write_u8(c, (uint8_t)MPU_CONFIG, 0x03)) { note_bus_fail(c); return 0; }
      c->initStep = 6; return 1;
    case 6:
      if (!write_u8(c, (uint8_t)MPU_GYRO_CONFIG, (uint8_t)MPU_GYRO_FS_2000)) { note_bus_fail(c); return 0; }
      c->initStep = 7; return 1;
    case 7:
      if (!write_u8(c, (uint8_t)MPU_ACCEL_CONFIG, (uint8_t)MPU_ACCEL_FS_16G)) { note_bus_fail(c); return 0; }
      c->state = IMU_DRV_SELFTEST; c->selftestCount = 0; c->waitTicks = 0; c->initStep = 0;
      return 1;
    default: c->initStep = 0; return 0;
  }
}

static int step_selftest(Mpu6050Core* c, MpuSampleOut* out, double t_s) {
  int16_t a[3], g[3]; float acc[3], gyr[3], mag;
  if (!read_raw(c, a, g)) { note_bus_fail(c); emit_invalid(c, out, t_s); return 1; }
  convert(a, g, acc, gyr);
  mag = sqrtf(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
  if (mag >= 0.5f*G0_MPS2 && mag <= 1.5f*G0_MPS2) c->selftestCount++;
  else if (c->selftestCount > 0) c->selftestCount--;
  emit_invalid(c, out, t_s);
  if (c->selftestCount >= c->params.selftestSamples) {
    c->state = IMU_DRV_READY; c->haveLast = 0; c->stuckRun = 0;
  } else if (++c->waitTicks > 2000) {
    enter_fault(c, IMU_FAULT_SELFTEST);
  }
  return 1;
}

static int step_ready(Mpu6050Core* c, MpuSampleOut* out, double t_s) {
  int16_t a[3], g[3]; float acc[3], gyr[3]; int i; int same;
  bool gyroSat = false, accelSat = false;
  if (!read_raw(c, a, g)) { note_bus_fail(c); emit_invalid(c, out, t_s); return 1; }
  convert(a, g, acc, gyr);
  same = c->haveLast &&
    acc[0]==c->lastA[0] && acc[1]==c->lastA[1] && acc[2]==c->lastA[2] &&
    gyr[0]==c->lastG[0] && gyr[1]==c->lastG[1] && gyr[2]==c->lastG[2];
  if (same) {
    c->stuckRun++;
    if (c->stuckRun >= c->params.stuckN) {
      c->stuckCount++; enter_fault(c, IMU_FAULT_STUCK); emit_invalid(c, out, t_s); return 1;
    }
  } else c->stuckRun = 0;
  for (i = 0; i < 3; ++i) { c->lastA[i] = acc[i]; c->lastG[i] = gyr[i]; }
  c->haveLast = 1;
  for (i = 0; i < 3; ++i) {
    if (fabsf(gyr[i]) >= c->params.gyroSatThresh_rads) gyroSat = true;
    if (fabsf(acc[i]) >= c->params.accelSatThresh_mps2) accelSat = true;
  }
  c->seq++;
  out->t_s = t_s; out->seq = c->seq; out->valid = true;
  out->accel_sensor_mps2[0]=acc[0]; out->accel_sensor_mps2[1]=acc[1]; out->accel_sensor_mps2[2]=acc[2];
  out->gyro_sensor_rads[0]=gyr[0]; out->gyro_sensor_rads[1]=gyr[1]; out->gyro_sensor_rads[2]=gyr[2];
  out->gyroSat = gyroSat; out->accelSat = accelSat;
  return 1;
}

bool mpu_core_tick(Mpu6050Core* c, double t_s, MpuSampleOut* out) {
  if (!c || !out) return false;
  memset(out, 0, sizeof(*out));
  switch (c->state) {
    case IMU_DRV_OFF:
      c->state = IMU_DRV_INIT; c->initStep = 0; c->waitTicks = 0;
      emit_invalid(c, out, t_s); return true;
    case IMU_DRV_INIT:
      step_init(c); emit_invalid(c, out, t_s); return true;
    case IMU_DRV_SELFTEST: return step_selftest(c, out, t_s) ? true : false;
    case IMU_DRV_READY: return step_ready(c, out, t_s) ? true : false;
    case IMU_DRV_FAULT: emit_invalid(c, out, t_s); return true;
  }
  return false;
}
