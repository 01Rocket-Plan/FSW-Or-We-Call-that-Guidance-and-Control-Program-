/* lsm6dso_core — ST LSM6DSO（6 軸 IMU）純 C 核心【備援分支，不進 main】
 * 跟 main 的 imu_core（ICM-20948）同一套狀態機與故障碼，F´ 外殼可互換。
 * 暫存器／數值來源：ST 官方驅動 github.com/STMicroelectronics/lsm6dso-pid（lsm6dso_reg.h/.c）
 *                 ＋ ST datasheet DS12140 Rev 3（Table 2 自測極限）。 */
#ifndef ENV_SENSING_LSM6DSO_CORE_H
#define ENV_SENSING_LSM6DSO_CORE_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 數值跟 RocketTypes.DrvState、imu_core 的 ImuDrvState 一致 */
typedef enum {
  LSM_DRV_OFF = 0,
  LSM_DRV_INIT = 1,
  LSM_DRV_SELFTEST = 2,
  LSM_DRV_READY = 3,
  LSM_DRV_FAULT = 4
} LsmDrvState;

/* 數值跟 imu_core 的 ImuFaultReason 一致（遙測 faultReason 意義不變） */
typedef enum {
  LSM_FAULT_NONE = 0,
  LSM_FAULT_WHOAMI = 1,
  LSM_FAULT_BUS = 2,
  LSM_FAULT_STUCK = 3,
  LSM_FAULT_SELFTEST = 4
} LsmFaultReason;

/* I2C 7-bit 位址：SDO/SA0 接地 0x6A、接 VDD 0x6B（ST：LSM6DSO_I2C_ADD_L/H = 0xD5/0xD7 為 8-bit 讀位址） */
#define LSM6DSO_I2C_ADDR_SA0_LOW  0x6Au
#define LSM6DSO_I2C_ADDR_SA0_HIGH 0x6Bu
#define LSM6DSO_WHO_AM_I_VALUE    0x6Cu

typedef struct {
  uint32_t stuckN;              /* 連續完全相同樣本數門檻 */
  uint32_t busErrThreshold;
  uint32_t whoamiFailThreshold;
  uint32_t selftestSamples;     /* 墊上靜置檢查要連續幾筆合格 */
  float gyroSatThresh_rads;
  float accelSatThresh_mps2;
  /* 機體←晶片 DCM（row-major）。機體系 X 鼻／Y 右／Z=X×Y（docs/conventions.md §2）。
   * 預設＝晶片 Z 沿鼻：機體 = (晶片 Z, 晶片 Y, −晶片 X)。實際安裝確定後要重設。 */
  float dcm[9];
  uint8_t i2cAddr;
} LsmParams;

typedef struct {
  double t_s;
  uint32_t seq;
  bool valid;
  float acc_mps2[3];
  float gyro_rads[3];
  bool gyroSat;
  bool accelSat;
} LsmSampleOut;

typedef struct {
  bool (*writeReg)(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
  bool (*readReg)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len);
  void* ctx;
} LsmBusOps;

typedef struct LsmCore LsmCore;

LsmCore* lsm_core_create(LsmBusOps bus);
void lsm_core_destroy(LsmCore* c);
void lsm_core_set_params(LsmCore* c, const LsmParams* p);
void lsm_core_reset(LsmCore* c);
bool lsm_core_tick(LsmCore* c, double t_s, LsmSampleOut* out);
LsmDrvState lsm_core_state(const LsmCore* c);
LsmFaultReason lsm_core_fault(const LsmCore* c);
uint32_t lsm_core_seq(const LsmCore* c);
uint32_t lsm_core_bus_err(const LsmCore* c);
uint32_t lsm_core_stuck(const LsmCore* c);
LsmParams lsm_params_default(void);

#ifdef __cplusplus
}
#endif
#endif
