#include "imu_core.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

enum {
  REG_BANK_SEL = 0x7F,
  BANK0_WHO_AM_I = 0x00,
  BANK0_PWR_MGMT_1 = 0x06,
  BANK0_PWR_MGMT_2 = 0x07,
  BANK0_ACCEL_XOUT_H = 0x2D,
  BANK1_ST_X_GYRO = 0x02,
  BANK1_ST_Y_GYRO = 0x03,
  BANK1_ST_Z_GYRO = 0x04,
  BANK1_ST_X_ACCEL = 0x0E,
  BANK1_ST_Y_ACCEL = 0x0F,
  BANK1_ST_Z_ACCEL = 0x10,
  BANK2_GYRO_CONFIG_1 = 0x01,
  BANK2_GYRO_CONFIG_2 = 0x02,
  BANK2_GYRO_SMPLRT_DIV = 0x00,
  BANK2_ACCEL_CONFIG = 0x14,
  BANK2_ACCEL_CONFIG_2 = 0x15,
  BANK2_ACCEL_SMPLRT_DIV_1 = 0x10,
  BANK2_ACCEL_SMPLRT_DIV_2 = 0x11,
  WHOAMI_ICM20948 = 0xEA,
  WHOAMI_ICM20648 = 0xE0,
  PWR1_DEVICE_RESET = 0x80,
  PWR1_CLKSEL_AUTO = 0x01,
  GYRO_CFG1_2000DPS_DLPF = 0x07,
  ACCEL_CFG_16G_DLPF = 0x07,
  GYRO_CFG1_250DPS_DLPF = 0x01,
  ACCEL_CFG_2G_DLPF = 0x01,
  GYRO_ST_ENABLE_XYZ = 0x38,
  ACCEL_ST_ENABLE_XYZ = 0x1C
};
static const float GYRO_SENS_2000_DPS = 16.4f;
static const float ACCEL_SENS_16G = 2048.0f;
static const float DEG2RAD = 0.017453292519943295f;
static const float G0_MPS2 = 9.80665f;
/* ST pass band vs factory trim: 50%–150% (datasheet uses MotionApps; OTP ratio gate) */
static const float ST_RATIO_MIN = 0.5f;
static const float ST_RATIO_MAX = 1.5f;

enum {
  ST_CFG_FS = 0,
  ST_WAIT_FS = 1,
  ST_READ_OTP = 2,
  ST_ACCUM_OFF = 3,
  ST_ENABLE = 4,
  ST_WAIT_ON = 5,
  ST_ACCUM_ON = 6,
  ST_EVAL = 7,
  ST_RESTORE_FS = 8,
  ST_REST_POSE = 9
};

struct ImuCore {
  ImuBusOps bus;
  ImuParams params;
  ImuDrvState state;
  ImuSensorId active;
  ImuFaultReason faultReason;
  uint8_t initStep;
  uint8_t stPhase;
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
  uint8_t stCodeA[3];
  uint8_t stCodeG[3];
  int32_t avgOffA[3];
  int32_t avgOffG[3];
  int32_t avgOnA[3];
  int32_t avgOnG[3];
  uint32_t avgN;
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
  c->stPhase = 0;
  c->waitTicks = 0;
  c->selftestCount = 0;
  c->whoamiFails = 0;
  c->stuckRun = 0;
  c->faultReason = IMU_FAULT_NONE;
  c->haveLast = 0;
  c->primaryFailed = 0;
  c->avgN = 0;
  memset(c->avgOffA, 0, sizeof(c->avgOffA));
  memset(c->avgOffG, 0, sizeof(c->avgOffG));
  memset(c->avgOnA, 0, sizeof(c->avgOnA));
  memset(c->avgOnG, 0, sizeof(c->avgOnG));
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
    c->stPhase = 0;
    c->waitTicks = 0;
    c->selftestCount = 0;
    c->whoamiFails = 0;
    c->busErrCount = 0;
    c->stuckRun = 0;
    c->haveLast = 0;
    c->avgN = 0;
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

static float factory_trim(uint8_t code) {
  /* FS=0: FT = 2620 * 1.01^(code-1); code 0 → no OTP, treat as N/A */
  if (code == 0) return 0.0f;
  return 2620.0f * powf(1.01f, (float)code - 1.0f);
}

static int axis_st_ok(uint8_t code, int32_t resp) {
  float ft = factory_trim(code);
  float ratio;
  if (ft <= 0.0f) {
    /* No OTP: require non-trivial absolute response (LSB at FS=0) */
    return abs(resp) >= 200;
  }
  ratio = (float)resp / ft;
  if (ratio < 0.0f) ratio = -ratio;
  return ratio >= ST_RATIO_MIN && ratio <= ST_RATIO_MAX;
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
      c->stPhase = ST_CFG_FS;
      c->selftestCount = 0;
      c->waitTicks = 0;
      c->avgN = 0;
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
  int i;
  uint32_t need = c->params.selftestSamples;
  if (need < 1) need = 1;

  emit_invalid(c, out, t_s);

  switch (c->stPhase) {
    case ST_CFG_FS:
      /* Switch to FS=0 (±250 dps / ±2 g) for factory ST */
      if (!select_bank(c, 2) ||
          !write_u8(c, (uint8_t)BANK2_GYRO_CONFIG_1, (uint8_t)GYRO_CFG1_250DPS_DLPF) ||
          !write_u8(c, (uint8_t)BANK2_ACCEL_CONFIG, (uint8_t)ACCEL_CFG_2G_DLPF) ||
          !write_u8(c, (uint8_t)BANK2_GYRO_CONFIG_2, 0x00) ||
          !write_u8(c, (uint8_t)BANK2_ACCEL_CONFIG_2, 0x00)) {
        note_bus_fail(c); return 1;
      }
      c->waitTicks = 20;
      c->stPhase = ST_WAIT_FS;
      return 1;

    case ST_WAIT_FS:
      if (c->waitTicks > 0) { c->waitTicks--; return 1; }
      c->stPhase = ST_READ_OTP;
      return 1;

    case ST_READ_OTP:
      if (!select_bank(c, 1)) { note_bus_fail(c); return 1; }
      if (!read_u8(c, (uint8_t)BANK1_ST_X_GYRO, &c->stCodeG[0]) ||
          !read_u8(c, (uint8_t)BANK1_ST_Y_GYRO, &c->stCodeG[1]) ||
          !read_u8(c, (uint8_t)BANK1_ST_Z_GYRO, &c->stCodeG[2]) ||
          !read_u8(c, (uint8_t)BANK1_ST_X_ACCEL, &c->stCodeA[0]) ||
          !read_u8(c, (uint8_t)BANK1_ST_Y_ACCEL, &c->stCodeA[1]) ||
          !read_u8(c, (uint8_t)BANK1_ST_Z_ACCEL, &c->stCodeA[2])) {
        note_bus_fail(c); return 1;
      }
      memset(c->avgOffA, 0, sizeof(c->avgOffA));
      memset(c->avgOffG, 0, sizeof(c->avgOffG));
      c->avgN = 0;
      c->stPhase = ST_ACCUM_OFF;
      return 1;

    case ST_ACCUM_OFF:
      if (!read_raw(c, a, g)) { note_bus_fail(c); return 1; }
      for (i = 0; i < 3; ++i) { c->avgOffA[i] += a[i]; c->avgOffG[i] += g[i]; }
      c->avgN++;
      if (c->avgN >= need) {
        for (i = 0; i < 3; ++i) {
          c->avgOffA[i] /= (int32_t)need;
          c->avgOffG[i] /= (int32_t)need;
        }
        c->stPhase = ST_ENABLE;
      }
      return 1;

    case ST_ENABLE:
      if (!select_bank(c, 2) ||
          !write_u8(c, (uint8_t)BANK2_GYRO_CONFIG_2, (uint8_t)GYRO_ST_ENABLE_XYZ) ||
          !write_u8(c, (uint8_t)BANK2_ACCEL_CONFIG_2, (uint8_t)ACCEL_ST_ENABLE_XYZ)) {
        note_bus_fail(c); return 1;
      }
      c->waitTicks = 50;
      c->stPhase = ST_WAIT_ON;
      return 1;

    case ST_WAIT_ON:
      if (c->waitTicks > 0) { c->waitTicks--; return 1; }
      memset(c->avgOnA, 0, sizeof(c->avgOnA));
      memset(c->avgOnG, 0, sizeof(c->avgOnG));
      c->avgN = 0;
      c->stPhase = ST_ACCUM_ON;
      return 1;

    case ST_ACCUM_ON:
      if (!read_raw(c, a, g)) { note_bus_fail(c); return 1; }
      for (i = 0; i < 3; ++i) { c->avgOnA[i] += a[i]; c->avgOnG[i] += g[i]; }
      c->avgN++;
      if (c->avgN >= need) {
        for (i = 0; i < 3; ++i) {
          c->avgOnA[i] /= (int32_t)need;
          c->avgOnG[i] /= (int32_t)need;
        }
        c->stPhase = ST_EVAL;
      }
      return 1;

    case ST_EVAL: {
      int ok = 1;
      for (i = 0; i < 3; ++i) {
        if (!axis_st_ok(c->stCodeA[i], c->avgOnA[i] - c->avgOffA[i])) ok = 0;
        if (!axis_st_ok(c->stCodeG[i], c->avgOnG[i] - c->avgOffG[i])) ok = 0;
      }
      /* Disable ST actuators */
      if (!select_bank(c, 2) ||
          !write_u8(c, (uint8_t)BANK2_GYRO_CONFIG_2, 0x00) ||
          !write_u8(c, (uint8_t)BANK2_ACCEL_CONFIG_2, 0x00)) {
        note_bus_fail(c); return 1;
      }
      if (!ok) {
        try_failover(c, IMU_FAULT_SELFTEST);
        return 1;
      }
      c->stPhase = ST_RESTORE_FS;
      return 1;
    }

    case ST_RESTORE_FS:
      if (!select_bank(c, 2) ||
          !write_u8(c, (uint8_t)BANK2_GYRO_CONFIG_1, (uint8_t)GYRO_CFG1_2000DPS_DLPF) ||
          !write_u8(c, (uint8_t)BANK2_ACCEL_CONFIG, (uint8_t)ACCEL_CFG_16G_DLPF)) {
        note_bus_fail(c); return 1;
      }
      c->waitTicks = 20;
      c->selftestCount = 0;
      c->stPhase = ST_REST_POSE;
      return 1;

    case ST_REST_POSE:
      if (c->waitTicks > 0) { c->waitTicks--; return 1; }
      if (!read_raw(c, a, g)) { note_bus_fail(c); return 1; }
      convert_dcm(c, a, g, acc, gyro);
      mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
      if (mag >= 0.5f*G0_MPS2 && mag <= 1.5f*G0_MPS2 &&
          fabsf(gyro[0]) < 1.0f && fabsf(gyro[1]) < 1.0f && fabsf(gyro[2]) < 1.0f) {
        c->selftestCount++;
      } else if (c->selftestCount > 0) {
        c->selftestCount--;
      }
      if (c->selftestCount >= need) {
        c->state = IMU_DRV_READY;
        c->stuckRun = 0;
        c->haveLast = 0;
        c->waitTicks = 0;
        c->stPhase = 0;
      } else {
        c->waitTicks = 0;
        /* reuse waitTicks as rest-pose timeout counter via avgN */
        c->avgN++;
        if (c->avgN > 2000) {
          c->avgN = 0;
          try_failover(c, IMU_FAULT_SELFTEST);
        }
      }
      return 1;

    default:
      c->stPhase = ST_CFG_FS;
      return 1;
  }
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
