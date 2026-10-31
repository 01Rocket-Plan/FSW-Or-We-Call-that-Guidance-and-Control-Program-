#include "imu_core.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Use C-compatible register constants from a tiny local copy to avoid C++ header */
enum {
  REG_BANK_SEL = 0x7F,
  BANK0_WHO_AM_I = 0x00,
  BANK0_PWR_MGMT_1 = 0x06,
  BANK0_PWR_MGMT_2 = 0x07,
  BANK0_ACCEL_XOUT_H = 0x2D,
  BANK2_GYRO_CONFIG_1 = 0x01,
  BANK2_GYRO_SMPLRT_DIV = 0x00,
  BANK2_ACCEL_CONFIG = 0x14,
  BANK2_ACCEL_SMPLRT_DIV_1 = 0x10,
  BANK2_ACCEL_SMPLRT_DIV_2 = 0x11,
  WHOAMI_ICM20948 = 0xEA,
  WHOAMI_ICM20648 = 0xE0,
  PWR1_DEVICE_RESET = 0x80,
  PWR1_CLKSEL_AUTO = 0x01,
  GYRO_CFG1_2000DPS_DLPF = 0x07,
  ACCEL_CFG_16G_DLPF = 0x07
};
static const float GYRO_SENS_2000_DPS = 16.4f;
static const float ACCEL_SENS_16G = 2048.0f;
static const float DEG2RAD = 0.017453292519943295f;
static const float G0_MPS2 = 9.80665f;

struct ImuCore {
  ImuBusOps bus;
  ImuParams params;
  ImuDrvState state;
  ImuSensorId active;
  ImuFaultReason faultReason;
  uint8_t initStep;
  uint32_t waitTicks;
  uint32_t selftestCount;
  uint32_t whoamiFails;
  uint32_t busErrCount;
  uint32_t stuckCount;
  uint32_t stuckRun;
  uint32_t failoverCount;
  uint32_t seq;
  int primaryFailed;
  float lastAcc[3];
  float lastGyro[3];
  int haveLast;
};

ImuParams imu_params_default(void) {
  ImuParams p;
  memset(&p, 0, sizeof(p));
  p.stuckN = 50;
  p.busErrThreshold = 5;
  p.whoamiFailThreshold = 3;
  p.selftestSamples = 20;
  p.gyroSatThresh_rads = 34.0f;
  p.accelSatThresh_mps2 = 156.0f;
  p.dcmPrimary[0] = p.dcmPrimary[4] = p.dcmPrimary[8] = 1.0f;
  p.dcmBackup[0] = p.dcmBackup[4] = p.dcmBackup[8] = 1.0f;
  p.primaryAddr = 0x69;
  p.backupAddr = 0x68;
  return p;
}

ImuCore* imu_core_create(ImuBusOps bus) {
  ImuCore* c = (ImuCore*)calloc(1, sizeof(ImuCore));
  if (!c) return NULL;
  c->bus = bus;
  c->params = imu_params_default();
  c->state = IMU_DRV_OFF;
  c->active = IMU_SENS_PRIMARY;
  return c;
}

void imu_core_destroy(ImuCore* c) { free(c); }

void imu_core_set_params(ImuCore* c, const ImuParams* p) {
  if (c && p) c->params = *p;
}

void imu_core_reset(ImuCore* c) {
  if (!c) return;
  c->state = IMU_DRV_OFF;
  c->active = IMU_SENS_PRIMARY;
  c->initStep = 0;
  c->waitTicks = 0;
  c->selftestCount = 0;
  c->whoamiFails = 0;
  c->stuckRun = 0;
  c->faultReason = IMU_FAULT_NONE;
  c->haveLast = 0;
  c->primaryFailed = 0;
}

ImuDrvState imu_core_state(const ImuCore* c) { return c->state; }
ImuSensorId imu_core_active(const ImuCore* c) { return c->active; }
ImuFaultReason imu_core_fault(const ImuCore* c) { return c->faultReason; }
uint32_t imu_core_seq(const ImuCore* c) { return c->seq; }
uint32_t imu_core_bus_err(const ImuCore* c) { return c->busErrCount; }
uint32_t imu_core_stuck(const ImuCore* c) { return c->stuckCount; }
uint32_t imu_core_failover(const ImuCore* c) { return c->failoverCount; }

static uint8_t addr_of(const ImuCore* c) {
  return c->active == IMU_SENS_PRIMARY ? c->params.primaryAddr : c->params.backupAddr;
}
static uint8_t expect_whoami(const ImuCore* c) {
  return c->active == IMU_SENS_PRIMARY ? (uint8_t)WHOAMI_ICM20948 : (uint8_t)WHOAMI_ICM20648;
}
static const float* dcm_of(const ImuCore* c) {
  return c->active == IMU_SENS_PRIMARY ? c->params.dcmPrimary : c->params.dcmBackup;
}

static void matvec3(const float m[9], const float v[3], float o[3]) {
  o[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
  o[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
  o[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

static int select_bank(ImuCore* c, uint8_t bank) {
  uint8_t v = (uint8_t)((bank & 3u) << 4);
  return c->bus.writeReg(c->bus.ctx, addr_of(c), (uint8_t)REG_BANK_SEL, &v, 1);
}
static int write_u8(ImuCore* c, uint8_t reg, uint8_t val) {
  return c->bus.writeReg(c->bus.ctx, addr_of(c), reg, &val, 1);
}
static int read_u8(ImuCore* c, uint8_t reg, uint8_t* val) {
  return c->bus.readReg(c->bus.ctx, addr_of(c), reg, val, 1);
}

static int read_raw(ImuCore* c, int16_t accel[3], int16_t gyro[3]) {
  uint8_t buf[12];
  int i;
  if (!select_bank(c, 0)) return 0;
  if (!c->bus.readReg(c->bus.ctx, addr_of(c), (uint8_t)BANK0_ACCEL_XOUT_H, buf, 12)) return 0;
  for (i = 0; i < 3; ++i) {
    accel[i] = (int16_t)((buf[2*i] << 8) | buf[2*i+1]);
    gyro[i]  = (int16_t)((buf[6+2*i] << 8) | buf[6+2*i+1]);
  }
  return 1;
}

static void convert_dcm(ImuCore* c, const int16_t accel[3], const int16_t gyro[3],
                        float acc_mps2[3], float gyro_rads[3]) {
  float as[3], gs[3];
  int i;
  for (i = 0; i < 3; ++i) {
    as[i] = (accel[i] / ACCEL_SENS_16G) * G0_MPS2;
    gs[i] = (gyro[i] / GYRO_SENS_2000_DPS) * DEG2RAD;
  }
  matvec3(dcm_of(c), as, acc_mps2);
  matvec3(dcm_of(c), gs, gyro_rads);
}

static void enter_fault(ImuCore* c, ImuFaultReason r) {
  c->state = IMU_DRV_FAULT;
  c->faultReason = r;
}

static void try_failover(ImuCore* c, ImuFaultReason r) {
  if (c->active == IMU_SENS_PRIMARY && !c->primaryFailed) {
    c->primaryFailed = 1;
    c->failoverCount++;
    c->active = IMU_SENS_BACKUP;
    c->state = IMU_DRV_INIT;
    c->initStep = 0;
    c->waitTicks = 0;
    c->selftestCount = 0;
    c->whoamiFails = 0;
    c->busErrCount = 0;
    c->stuckRun = 0;
    c->haveLast = 0;
    c->faultReason = IMU_FAULT_NONE;
    return;
  }
  enter_fault(c, r == IMU_FAULT_NONE ? IMU_FAULT_BOTH : r);
}

static void note_bus_fail(ImuCore* c) {
  c->busErrCount++;
  if (c->busErrCount >= c->params.busErrThreshold) try_failover(c, IMU_FAULT_BUS);
}

static void emit_invalid(ImuCore* c, ImuSampleOut* out, double t_s) {
  memset(out, 0, sizeof(*out));
  out->t_s = t_s;
  out->seq = c->seq;
  out->valid = false;
}

static int step_init(ImuCore* c) {
  uint8_t id;
  switch (c->initStep) {
    case 0:
      if (!select_bank(c, 0) || !write_u8(c, (uint8_t)BANK0_PWR_MGMT_1, (uint8_t)PWR1_DEVICE_RESET)) {
        note_bus_fail(c); return 0;
      }
      c->waitTicks = 10;
      c->initStep = 1;
      return 1;
    case 1:
      if (c->waitTicks > 0) { c->waitTicks--; return 1; }
      c->initStep = 2;
      return 1;
    case 2:
      if (!select_bank(c, 0)) { note_bus_fail(c); return 0; }
      if (!read_u8(c, (uint8_t)BANK0_WHO_AM_I, &id)) { note_bus_fail(c); return 0; }
      if (id != expect_whoami(c)) {
        c->whoamiFails++;
        if (c->whoamiFails >= c->params.whoamiFailThreshold) try_failover(c, IMU_FAULT_WHOAMI);
        return 0;
      }
      c->initStep = 3;
      return 1;
    case 3:
      if (!write_u8(c, (uint8_t)BANK0_PWR_MGMT_1, (uint8_t)PWR1_CLKSEL_AUTO)) { note_bus_fail(c); return 0; }
      c->initStep = 4; return 1;
    case 4:
      if (!write_u8(c, (uint8_t)BANK0_PWR_MGMT_2, 0x00)) { note_bus_fail(c); return 0; }
      c->initStep = 5; return 1;
    case 5:
      if (!select_bank(c, 2) || !write_u8(c, (uint8_t)BANK2_GYRO_CONFIG_1, (uint8_t)GYRO_CFG1_2000DPS_DLPF)) {
        note_bus_fail(c); return 0;
      }
      c->initStep = 6; return 1;
    case 6:
      if (!write_u8(c, (uint8_t)BANK2_GYRO_SMPLRT_DIV, 0x00)) { note_bus_fail(c); return 0; }
      c->initStep = 7; return 1;
    case 7:
      if (!write_u8(c, (uint8_t)BANK2_ACCEL_CONFIG, (uint8_t)ACCEL_CFG_16G_DLPF)) { note_bus_fail(c); return 0; }
      c->initStep = 8; return 1;
    case 8:
      if (!write_u8(c, (uint8_t)BANK2_ACCEL_SMPLRT_DIV_1, 0x00) ||
          !write_u8(c, (uint8_t)BANK2_ACCEL_SMPLRT_DIV_2, 0x00)) {
        note_bus_fail(c); return 0;
      }
      c->initStep = 9; return 1;
    case 9:
      if (!select_bank(c, 0)) { note_bus_fail(c); return 0; }
      c->state = IMU_DRV_SELFTEST;
      c->selftestCount = 0;
      c->initStep = 0;
      return 1;
    default:
      c->initStep = 0;
      return 0;
  }
}

static void check_stuck_sat(ImuCore* c, const float acc[3], const float gyro[3],
                            bool* gyroSat, bool* accelSat, bool* stuck) {
  int i;
  *gyroSat = false; *accelSat = false; *stuck = false;
  for (i = 0; i < 3; ++i) {
    if (fabsf(gyro[i]) >= c->params.gyroSatThresh_rads) *gyroSat = true;
    if (fabsf(acc[i]) >= c->params.accelSatThresh_mps2) *accelSat = true;
  }
  if (c->haveLast &&
      acc[0]==c->lastAcc[0] && acc[1]==c->lastAcc[1] && acc[2]==c->lastAcc[2] &&
      gyro[0]==c->lastGyro[0] && gyro[1]==c->lastGyro[1] && gyro[2]==c->lastGyro[2]) {
    c->stuckRun++;
    if (c->stuckRun >= c->params.stuckN) { *stuck = true; c->stuckCount++; }
  } else {
    c->stuckRun = 0;
  }
  for (i = 0; i < 3; ++i) { c->lastAcc[i] = acc[i]; c->lastGyro[i] = gyro[i]; }
  c->haveLast = 1;
}

static int step_selftest(ImuCore* c, ImuSampleOut* out, double t_s) {
  int16_t a[3], g[3];
  float acc[3], gyro[3], mag;
  if (!read_raw(c, a, g)) { note_bus_fail(c); emit_invalid(c, out, t_s); return 1; }
  convert_dcm(c, a, g, acc, gyro);
  mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
  if (mag >= 0.5f*G0_MPS2 && mag <= 1.5f*G0_MPS2 &&
      fabsf(gyro[0]) < 1.0f && fabsf(gyro[1]) < 1.0f && fabsf(gyro[2]) < 1.0f) {
    c->selftestCount++;
  } else if (c->selftestCount > 0) {
    c->selftestCount--;
  }
  emit_invalid(c, out, t_s);
  if (c->selftestCount >= c->params.selftestSamples) {
    c->state = IMU_DRV_READY;
    c->stuckRun = 0;
    c->haveLast = 0;
    c->waitTicks = 0;
  } else {
    c->waitTicks++;
    if (c->waitTicks > 2000) {
      c->waitTicks = 0;
      try_failover(c, IMU_FAULT_SELFTEST);
    }
  }
  return 1;
}

static int step_ready(ImuCore* c, ImuSampleOut* out, double t_s) {
  int16_t a[3], g[3];
  float acc[3], gyro[3];
  bool gyroSat, accelSat, stuck;
  if (!read_raw(c, a, g)) { note_bus_fail(c); emit_invalid(c, out, t_s); return 1; }
  convert_dcm(c, a, g, acc, gyro);
  check_stuck_sat(c, acc, gyro, &gyroSat, &accelSat, &stuck);
  if (stuck) {
    try_failover(c, IMU_FAULT_STUCK);
    emit_invalid(c, out, t_s);
    return 1;
  }
  c->seq++;
  out->t_s = t_s;
  out->seq = c->seq;
  out->valid = true;
  out->acc_mps2[0] = acc[0]; out->acc_mps2[1] = acc[1]; out->acc_mps2[2] = acc[2];
  out->gyro_rads[0] = gyro[0]; out->gyro_rads[1] = gyro[1]; out->gyro_rads[2] = gyro[2];
  out->gyroSat = gyroSat;
  out->accelSat = accelSat;
  return 1;
}

bool imu_core_tick(ImuCore* c, double t_s, ImuSampleOut* out) {
  if (!c || !out) return false;
  memset(out, 0, sizeof(*out));
  switch (c->state) {
    case IMU_DRV_OFF:
      c->state = IMU_DRV_INIT; c->initStep = 0; c->waitTicks = 0;
      emit_invalid(c, out, t_s); return true;
    case IMU_DRV_INIT:
      step_init(c); emit_invalid(c, out, t_s); return true;
    case IMU_DRV_SELFTEST:
      return step_selftest(c, out, t_s) ? true : false;
    case IMU_DRV_READY:
      return step_ready(c, out, t_s) ? true : false;
    case IMU_DRV_FAULT:
      emit_invalid(c, out, t_s); return true;
  }
  return false;
}
