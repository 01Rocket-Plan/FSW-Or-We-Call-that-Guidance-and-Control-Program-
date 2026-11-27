// ======================================================================
// \title  UsMpu6050Sim.hpp
// \brief  UsMpu6050 Sim — Base ID 0x4280
// ======================================================================
#ifndef COMPONENTS_USMPU6050SIM_HPP
#define COMPONENTS_USMPU6050SIM_HPP

#include "Components/EnvSensing/UsMpu6050/UsMpu6050SimComponentAc.hpp"
#include "Components/EnvSensing/UsMpu6050/MpuSimDevice.hpp"
#include "Components/EnvSensing/UsMpu6050/mpu6050_core.h"

namespace Components {

class UsMpu6050Sim final : public UsMpu6050SimComponentBase {
 public:
  explicit UsMpu6050Sim(const char* compName);
  ~UsMpu6050Sim() override;

  //! 餵入感測器原生軸原始值（轉給 MpuSimDevice::setTruth）
  void setTruth(const int16_t accel_raw[3], const int16_t gyro_raw[3]) {
    m_dev.setTruth(accel_raw, gyro_raw);
  }

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void injectFault_handler(FwIndexType portNum, U8 code) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void emitSample(const MpuSampleOut& s);

  MpuSimDevice m_dev;
  Mpu6050Core* m_core;
  U32 m_tlmDiv;
  ImuDrvState m_lastLoggedState;
};

}  // namespace Components
#endif
