#ifndef ENV_SENSING_IMU_CORE_H
#define ENV_SENSING_IMU_CORE_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  IMU_DRV_OFF = 0,
  IMU_DRV_INIT = 1,
  IMU_DRV_SELFTEST = 2,
  IMU_DRV_READY = 3,
  IMU_DRV_FAULT = 4
} ImuDrvState;

typedef enum {
  IMU_FAULT_NONE = 0,
  IMU_FAULT_WHOAMI = 1,
  IMU_FAULT_BUS = 2,
  IMU_FAULT_STUCK = 3,
  IMU_FAULT_SELFTEST = 4
} ImuFaultReason;

typedef struct {
  /* 整數型別可存範圍（無號）：
   *   U8  (uint8_t)  : 0 ~ 255
   *   U16 (uint16_t) : 0 ~ 65,535
   *   U32 (uint32_t) : 0 ~ 4,294,967,295（約 42 億）
   * 以下計數與門檻一律用 U32，避免溢位；stuckN 為連續相同樣本數門檻。 */
  uint32_t stuckN;
  uint32_t busErrThreshold;
  uint32_t whoamiFailThreshold;
  uint32_t selftestSamples;
  float gyroSatThresh_rads;
  float accelSatThresh_mps2;
  /* 機體←晶片 DCM（row-major）。機體系 X 鼻／Y 右／Z=X×Y（docs/conventions.md §2）。
   * 預設＝ICM-20948 晶片 Z 沿鼻的安裝：機體 = (晶片 Z, 晶片 Y, −晶片 X)。 */
  float dcm[9];
  uint8_t i2cAddr;
} ImuParams;

typedef struct {
  double t_s;
  uint32_t seq;
  bool valid;
  float acc_mps2[3];
  float gyro_rads[3];
  bool gyroSat;
  bool accelSat;
} ImuSampleOut;

typedef struct {
  bool (*writeReg)(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
  bool (*readReg)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len);
  void* ctx;
} ImuBusOps;

typedef struct ImuCore ImuCore;

ImuCore* imu_core_create(ImuBusOps bus);
void imu_core_destroy(ImuCore* c);
void imu_core_set_params(ImuCore* c, const ImuParams* p);
void imu_core_reset(ImuCore* c);
bool imu_core_tick(ImuCore* c, double t_s, ImuSampleOut* out);
ImuDrvState imu_core_state(const ImuCore* c);
ImuFaultReason imu_core_fault(const ImuCore* c);
uint32_t imu_core_seq(const ImuCore* c);
uint32_t imu_core_bus_err(const ImuCore* c);
uint32_t imu_core_stuck(const ImuCore* c);
ImuParams imu_params_default(void);

#ifdef __cplusplus
}
#endif
#endif
