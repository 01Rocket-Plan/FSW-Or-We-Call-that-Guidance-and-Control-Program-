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

typedef enum { IMU_SENS_PRIMARY = 0, IMU_SENS_BACKUP = 1 } ImuSensorId;
typedef enum {
  IMU_FAULT_NONE = 0,
  IMU_FAULT_WHOAMI = 1,
  IMU_FAULT_BUS = 2,
  IMU_FAULT_STUCK = 3,
  IMU_FAULT_SELFTEST = 4,
  IMU_FAULT_BOTH = 5
} ImuFaultReason;

typedef struct {
  uint32_t stuckN;
  uint32_t busErrThreshold;
  uint32_t whoamiFailThreshold;
  uint32_t selftestSamples;
  float gyroSatThresh_rads;
  float accelSatThresh_mps2;
  float dcmPrimary[9];
  float dcmBackup[9];
  uint8_t primaryAddr;
  uint8_t backupAddr;
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
ImuSensorId imu_core_active(const ImuCore* c);
ImuFaultReason imu_core_fault(const ImuCore* c);
uint32_t imu_core_seq(const ImuCore* c);
uint32_t imu_core_bus_err(const ImuCore* c);
uint32_t imu_core_stuck(const ImuCore* c);
uint32_t imu_core_failover(const ImuCore* c);
ImuParams imu_params_default(void);

#ifdef __cplusplus
}
#endif
#endif
