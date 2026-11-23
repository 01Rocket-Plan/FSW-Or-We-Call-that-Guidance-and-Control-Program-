/* lsm6dso_core.c — ST LSM6DSO 非阻塞狀態機（1 kHz 每拍呼叫一次 lsm_core_tick）
 *   OFF → INIT（SW_RESET、WHO_AM_I=0x6C、BDU/IF_INC）
 *       → SELFTEST（ST 內建自測：先加速度、後陀螺；再墊上靜置檢查）
 *       → READY（1667 Hz、±16 g、±2000 dps；DCM 轉機體系）／FAULT
 * 自測流程照 ST 官方範例 lsm6dso_self_test.c：設定→等 100 ms→丟第一筆→平均 5 筆（ST 關）
 *   →開 ST→等 100 ms→丟第一筆→平均 5 筆（ST 開）→ |開−關| 落在 datasheet 範圍內。 */
#include "lsm6dso_core.h"
#include "Lsm6dsoRegs.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const float DEG2RAD = 0.017453292519943295f;
static const float G0_MPS2 = 9.80665f;

enum { ST_SENSOR_XL = 0, ST_SENSOR_G = 1 };

enum {
  ST_CFG = 0,
  ST_WAIT_CFG = 1,
  ST_ACCUM_OFF = 2,
  ST_ENABLE = 3,
  ST_WAIT_ON = 4,
  ST_ACCUM_ON = 5,
  ST_EVAL = 6,
  ST_RESTORE = 7,
  ST_REST_POSE = 8
};

enum {
  BOOT_WAIT_TICKS = 10,     /* ST 範例 BOOT_TIME 10 ms */
  RESET_POLL_MAX = 50,      /* SW_RESET 位元 50 ms 內沒自己清掉 → 匯流排／晶片異常 */
  ST_SETTLE_TICKS = 100,    /* ST 範例 platform_delay(100) */
  ST_AVG_SAMPLES = 5,       /* ST 範例平均 5 筆 */
  ST_READ_EVERY_XL = 20,    /* 52 Hz ODR → 每 20 ms 讀一筆，保證是新資料 */
  ST_READ_EVERY_G = 5,      /* 208 Hz ODR → 每 5 ms 讀一筆 */
  RESTORE_SETTLE_TICKS = 100,
  REST_POSE_TIMEOUT = 2000
};

struct LsmCore {
  LsmBusOps bus;
  LsmParams params;
  LsmDrvState state;
  LsmFaultReason faultReason;
  uint8_t initStep;
  uint8_t stPhase;
  uint8_t stSensor;
  uint32_t waitTicks;
  uint32_t pollCount;
  uint32_t selftestCount;
  uint32_t whoamiFails;
  uint32_t busErrCount;
  uint32_t stuckCount;
  uint32_t stuckRun;
  uint32_t seq;
  float lastAcc[3];
  float lastGyro[3];
  int haveLast;
  int32_t sumOff[3];
  int32_t sumOn[3];
  uint32_t avgN;
  uint32_t restTicks;
};

LsmParams lsm_params_default(void) {
  LsmParams p;
  memset(&p, 0, sizeof(p));
  p.stuckN = 50;
  p.busErrThreshold = 5;
  p.whoamiFailThreshold = 3;
  p.selftestSamples = 20;
  p.gyroSatThresh_rads = 34.0f;    /* 跟 main 相同（額定 ±2000 dps ≈ 34.9 rad/s） */
  p.accelSatThresh_mps2 = 156.0f;  /* 跟 main 相同（±16 g ≈ 156.9 m/s²） */
  /* 機體 = (晶片 Z, 晶片 Y, −晶片 X)：墊上靜置比力 ≈ (+g, 0, 0)。實際安裝確定後要重設 dcm** 參數 */
  p.dcm[2] = 1.0f;
  p.dcm[4] = 1.0f;
  p.dcm[6] = -1.0f;
  p.i2cAddr = (uint8_t)LSM6DSO_I2C_ADDR_SA0_LOW;
  return p;
}

LsmCore* lsm_core_create(LsmBusOps bus) {
  LsmCore* c = (LsmCore*)calloc(1, sizeof(LsmCore));
  if (!c) return NULL;
  c->bus = bus;
  c->params = lsm_params_default();
  c->state = LSM_DRV_OFF;
  return c;
}

void lsm_core_destroy(LsmCore* c) { free(c); }

void lsm_core_set_params(LsmCore* c, const LsmParams* p) {
  if (c && p) c->params = *p;
}

void lsm_core_reset(LsmCore* c) {
  if (!c) return;
  c->state = LSM_DRV_OFF;
  c->faultReason = LSM_FAULT_NONE;
  c->initStep = 0;
  c->stPhase = ST_CFG;
  c->stSensor = ST_SENSOR_XL;
  c->waitTicks = 0;
  c->pollCount = 0;
  c->selftestCount = 0;
  c->whoamiFails = 0;
  c->stuckRun = 0;
  c->haveLast = 0;
  c->avgN = 0;
  c->restTicks = 0;
  memset(c->sumOff, 0, sizeof(c->sumOff));
  memset(c->sumOn, 0, sizeof(c->sumOn));
}

LsmDrvState lsm_core_state(const LsmCore* c) { return c->state; }
LsmFaultReason lsm_core_fault(const LsmCore* c) { return c->faultReason; }
uint32_t lsm_core_seq(const LsmCore* c) { return c->seq; }
uint32_t lsm_core_bus_err(const LsmCore* c) { return c->busErrCount; }
uint32_t lsm_core_stuck(const LsmCore* c) { return c->stuckCount; }

static uint8_t addr_of(const LsmCore* c) { return c->params.i2cAddr; }

static void matvec3(const float m[9], const float v[3], float o[3]) {
  o[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
  o[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
  o[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

static int write_u8(LsmCore* c, uint8_t reg, uint8_t val) {
  return c->bus.writeReg(c->bus.ctx, addr_of(c), reg, &val, 1);
}
static int read_u8(LsmCore* c, uint8_t reg, uint8_t* val) {
  return c->bus.readReg(c->bus.ctx, addr_of(c), reg, val, 1);
}

/* 一次連讀 12 bytes（IF_INC=1）：0x22 起陀螺 X/Y/Z，0x28 起加速度 X/Y/Z，每軸 L 在前 */
static int read_raw(LsmCore* c, int16_t accel[3], int16_t gyro[3]) {
  uint8_t buf[12];
  int i;
  if (!c->bus.readReg(c->bus.ctx, addr_of(c), (uint8_t)LSM_REG_OUTX_L_G, buf, 12)) return 0;
  for (i = 0; i < 3; ++i) {
    gyro[i]  = (int16_t)(uint16_t)(buf[2*i] | (buf[2*i+1] << 8));
    accel[i] = (int16_t)(uint16_t)(buf[6+2*i] | (buf[6+2*i+1] << 8));
  }
  return 1;
}

static void convert_dcm(LsmCore* c, const int16_t accel[3], const int16_t gyro[3],
                        float acc_mps2[3], float gyro_rads[3]) {
  float as[3], gs[3];
  int i;
  for (i = 0; i < 3; ++i) {
    as[i] = (float)accel[i] * LSM_SENS_16G_MG_PER_LSB * 1e-3f * G0_MPS2;
    gs[i] = (float)gyro[i] * LSM_SENS_2000DPS_MDPS_PER_LSB * 1e-3f * DEG2RAD;
  }
  matvec3(c->params.dcm, as, acc_mps2);
  matvec3(c->params.dcm, gs, gyro_rads);
}

static void enter_fault(LsmCore* c, LsmFaultReason r) {
  c->state = LSM_DRV_FAULT;
  c->faultReason = r;
}

static void note_bus_fail(LsmCore* c) {
  c->busErrCount++;
  if (c->busErrCount >= c->params.busErrThreshold) enter_fault(c, LSM_FAULT_BUS);
}

static void emit_invalid(LsmCore* c, LsmSampleOut* out, double t_s) {
  memset(out, 0, sizeof(*out));
  out->t_s = t_s;
  out->seq = c->seq;
  out->valid = false;
}

static void step_init(LsmCore* c) {
  uint8_t v;
  switch (c->initStep) {
    case 0:  /* 軟體重置 */
      if (!write_u8(c, (uint8_t)LSM_REG_CTRL3_C, (uint8_t)LSM_CTRL3_SW_RESET)) { note_bus_fail(c); return; }
      c->waitTicks = BOOT_WAIT_TICKS;
      c->pollCount = 0;
      c->initStep = 1;
      return;
    case 1:
      if (c->waitTicks > 0) { c->waitTicks--; return; }
      c->initStep = 2;
      return;
    case 2:  /* WHO_AM_I */
      if (!read_u8(c, (uint8_t)LSM_REG_WHO_AM_I, &v)) { note_bus_fail(c); return; }
      if (v != (uint8_t)LSM6DSO_WHO_AM_I_VALUE) {
        c->whoamiFails++;
        if (c->whoamiFails >= c->params.whoamiFailThreshold) enter_fault(c, LSM_FAULT_WHOAMI);
        return;
      }
      c->initStep = 3;
      return;
    case 3:  /* 等 SW_RESET 位元自己清掉（ST lsm6dso_reset_get 迴圈的非阻塞版） */
      if (!read_u8(c, (uint8_t)LSM_REG_CTRL3_C, &v)) { note_bus_fail(c); return; }
      if (v & (uint8_t)LSM_CTRL3_SW_RESET) {
        if (++c->pollCount >= RESET_POLL_MAX) enter_fault(c, LSM_FAULT_BUS);
        return;
      }
      c->initStep = 4;
      return;
    case 4:  /* BDU＋位址自動遞增（連讀 12 bytes 要用） */
      if (!write_u8(c, (uint8_t)LSM_REG_CTRL3_C, (uint8_t)(LSM_CTRL3_BDU | LSM_CTRL3_IF_INC))) {
        note_bus_fail(c); return;
      }
      c->initStep = 5;
      return;
    case 5:
      if (!write_u8(c, (uint8_t)LSM_REG_CTRL5_C, (uint8_t)LSM_CTRL5_ST_OFF)) { note_bus_fail(c); return; }
      c->state = LSM_DRV_SELFTEST;
      c->stSensor = ST_SENSOR_XL;
      c->stPhase = ST_CFG;
      c->selftestCount = 0;
      c->waitTicks = 0;
      c->avgN = 0;
      c->initStep = 0;
      return;
    default:
      c->initStep = 0;
      return;
  }
}

static void check_stuck_sat(LsmCore* c, const float acc[3], const float gyro[3],
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

/* 自測期間每 N 拍讀一筆；第 0 筆丟掉（ST 範例），之後累加 ST_AVG_SAMPLES 筆。回傳 1＝收滿 */
static int st_accumulate(LsmCore* c, int32_t sum[3]) {
  int16_t a[3], g[3];
  const int16_t* v;
  int i;
  const uint32_t every = (c->stSensor == ST_SENSOR_XL) ? ST_READ_EVERY_XL : ST_READ_EVERY_G;
  if (c->waitTicks > 0) { c->waitTicks--; return 0; }
  c->waitTicks = every - 1u;
  if (!read_raw(c, a, g)) { note_bus_fail(c); return 0; }
  v = (c->stSensor == ST_SENSOR_XL) ? a : g;
  if (c->avgN > 0) for (i = 0; i < 3; ++i) sum[i] += v[i];
  c->avgN++;
  return c->avgN > (uint32_t)ST_AVG_SAMPLES;
}

static int st_axis_ok(int sensor, int32_t sumOn, int32_t sumOff) {
  const float diffLsb = fabsf((float)(sumOn - sumOff) / (float)ST_AVG_SAMPLES);
  if (sensor == ST_SENSOR_XL) {
    const float mg = diffLsb * LSM_SENS_4G_MG_PER_LSB;
    return mg >= LSM_ST_XL_MIN_MG && mg <= LSM_ST_XL_MAX_MG;
  } else {
    const float mdps = diffLsb * LSM_SENS_2000DPS_MDPS_PER_LSB;
    return mdps >= LSM_ST_G_MIN_MDPS && mdps <= LSM_ST_G_MAX_MDPS;
  }
}

static void step_selftest(LsmCore* c, LsmSampleOut* out, double t_s) {
  int16_t a[3], g[3];
  float acc[3], gyro[3], mag;
  int i;
  uint32_t need = c->params.selftestSamples;
  if (need < 1) need = 1;

  emit_invalid(c, out, t_s);

  switch (c->stPhase) {
    case ST_CFG: {
      const int xl = (c->stSensor == ST_SENSOR_XL);
      if (!write_u8(c, (uint8_t)LSM_REG_CTRL1_XL, (uint8_t)(xl ? LSM_CTRL1_XL_52HZ_4G : LSM_CTRL_ODR_OFF)) ||
          !write_u8(c, (uint8_t)LSM_REG_CTRL2_G, (uint8_t)(xl ? LSM_CTRL_ODR_OFF : LSM_CTRL2_G_208HZ_2000DPS))) {
        note_bus_fail(c); return;
      }
      c->waitTicks = ST_SETTLE_TICKS;
      c->stPhase = ST_WAIT_CFG;
      return;
    }

    case ST_WAIT_CFG:
      if (c->waitTicks > 0) { c->waitTicks--; return; }
      memset(c->sumOff, 0, sizeof(c->sumOff));
      c->avgN = 0;
      c->stPhase = ST_ACCUM_OFF;
      return;

    case ST_ACCUM_OFF:
      if (st_accumulate(c, c->sumOff)) c->stPhase = ST_ENABLE;
      return;

    case ST_ENABLE:
      if (!write_u8(c, (uint8_t)LSM_REG_CTRL5_C,
                    (uint8_t)(c->stSensor == ST_SENSOR_XL ? LSM_CTRL5_ST_XL_POS : LSM_CTRL5_ST_G_POS))) {
        note_bus_fail(c); return;
      }
      c->waitTicks = ST_SETTLE_TICKS;
      c->stPhase = ST_WAIT_ON;
      return;

    case ST_WAIT_ON:
      if (c->waitTicks > 0) { c->waitTicks--; return; }
      memset(c->sumOn, 0, sizeof(c->sumOn));
      c->avgN = 0;
      c->stPhase = ST_ACCUM_ON;
      return;

    case ST_ACCUM_ON:
      if (st_accumulate(c, c->sumOn)) c->stPhase = ST_EVAL;
      return;

    case ST_EVAL: {
      int ok = 1;
      for (i = 0; i < 3; ++i) {
        if (!st_axis_ok(c->stSensor, c->sumOn[i], c->sumOff[i])) ok = 0;
      }
      if (!write_u8(c, (uint8_t)LSM_REG_CTRL5_C, (uint8_t)LSM_CTRL5_ST_OFF)) { note_bus_fail(c); return; }
      if (!ok) { enter_fault(c, LSM_FAULT_SELFTEST); return; }
      if (c->stSensor == ST_SENSOR_XL) {
        c->stSensor = ST_SENSOR_G;
        c->stPhase = ST_CFG;
      } else {
        c->stPhase = ST_RESTORE;
      }
      return;
    }

    case ST_RESTORE:  /* 回飛行設定：1667 Hz、±16 g、±2000 dps */
      if (!write_u8(c, (uint8_t)LSM_REG_CTRL1_XL, (uint8_t)LSM_CTRL1_XL_1667HZ_16G) ||
          !write_u8(c, (uint8_t)LSM_REG_CTRL2_G, (uint8_t)LSM_CTRL2_G_1667HZ_2000DPS)) {
        note_bus_fail(c); return;
      }
      c->waitTicks = RESTORE_SETTLE_TICKS;
      c->selftestCount = 0;
      c->restTicks = 0;
      c->stPhase = ST_REST_POSE;
      return;

    case ST_REST_POSE:  /* 墊上靜置：|a| 在 0.5–1.5 g、各軸角速度 < 1 rad/s，連續 selftestSamples 筆 */
      if (c->waitTicks > 0) { c->waitTicks--; return; }
      if (!read_raw(c, a, g)) { note_bus_fail(c); return; }
      convert_dcm(c, a, g, acc, gyro);
      mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
      if (mag >= 0.5f*G0_MPS2 && mag <= 1.5f*G0_MPS2 &&
          fabsf(gyro[0]) < 1.0f && fabsf(gyro[1]) < 1.0f && fabsf(gyro[2]) < 1.0f) {
        c->selftestCount++;
      } else if (c->selftestCount > 0) {
        c->selftestCount--;
      }
      if (c->selftestCount >= need) {
        c->state = LSM_DRV_READY;
        c->stuckRun = 0;
        c->haveLast = 0;
        c->stPhase = ST_CFG;
      } else if (++c->restTicks > REST_POSE_TIMEOUT) {
        enter_fault(c, LSM_FAULT_SELFTEST);
      }
      return;

    default:
      c->stPhase = ST_CFG;
      return;
  }
}

static void step_ready(LsmCore* c, LsmSampleOut* out, double t_s) {
  int16_t a[3], g[3];
  float acc[3], gyro[3];
  bool gyroSat, accelSat, stuck;
  if (!read_raw(c, a, g)) { note_bus_fail(c); emit_invalid(c, out, t_s); return; }
  convert_dcm(c, a, g, acc, gyro);
  check_stuck_sat(c, acc, gyro, &gyroSat, &accelSat, &stuck);
  if (stuck) {
    enter_fault(c, LSM_FAULT_STUCK);
    emit_invalid(c, out, t_s);
    return;
  }
  c->seq++;
  out->t_s = t_s;
  out->seq = c->seq;
  out->valid = true;
  for (int i = 0; i < 3; ++i) { out->acc_mps2[i] = acc[i]; out->gyro_rads[i] = gyro[i]; }
  out->gyroSat = gyroSat;
  out->accelSat = accelSat;
}

bool lsm_core_tick(LsmCore* c, double t_s, LsmSampleOut* out) {
  if (!c || !out) return false;
  memset(out, 0, sizeof(*out));
  switch (c->state) {
    case LSM_DRV_OFF:
      c->state = LSM_DRV_INIT; c->initStep = 0; c->waitTicks = 0;
      emit_invalid(c, out, t_s); return true;
    case LSM_DRV_INIT:
      step_init(c); emit_invalid(c, out, t_s); return true;
    case LSM_DRV_SELFTEST:
      step_selftest(c, out, t_s); return true;
    case LSM_DRV_READY:
      step_ready(c, out, t_s); return true;
    case LSM_DRV_FAULT:
      emit_invalid(c, out, t_s); return true;
  }
  return false;
}
