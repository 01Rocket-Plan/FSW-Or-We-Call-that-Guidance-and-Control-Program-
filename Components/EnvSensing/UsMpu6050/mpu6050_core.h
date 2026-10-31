#ifndef ENV_SENSING_MPU6050_CORE_H
#define ENV_SENSING_MPU6050_CORE_H
#include <stdint.h>
#include <stdbool.h>
#include "../ImuDriver/imu_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t stuckN;
  uint32_t busErrThreshold;
  uint32_t whoamiFailThreshold;
  uint32_t selftestSamples;
  float gyroSatThresh_rads;
  float accelSatThresh_mps2;
  uint8_t i2cAddr;
} MpuParams;

typedef struct {
  double t_s;
  uint32_t seq;
  bool valid;
  float accel_sensor_mps2[3];
  float gyro_sensor_rads[3];
  bool accelSat;
  bool gyroSat;
} MpuSampleOut;

typedef struct Mpu6050Core Mpu6050Core;

MpuParams mpu_params_default(void);
Mpu6050Core* mpu_core_create(ImuBusOps bus);
void mpu_core_destroy(Mpu6050Core* c);
void mpu_core_set_params(Mpu6050Core* c, const MpuParams* p);
void mpu_core_reset(Mpu6050Core* c);
bool mpu_core_tick(Mpu6050Core* c, double t_s, MpuSampleOut* out);
ImuDrvState mpu_core_state(const Mpu6050Core* c);
ImuFaultReason mpu_core_fault(const Mpu6050Core* c);
uint32_t mpu_core_seq(const Mpu6050Core* c);

#ifdef __cplusplus
}
#endif
#endif
