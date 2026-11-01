// ======================================================================
// \title  UsMpu6050.hpp
// \brief  US MPU6050 — Base ID 0x4200；Sim 0x4280
// ======================================================================
#ifndef COMPONENTS_USMPU6050_HPP
#define COMPONENTS_USMPU6050_HPP

#include "Components/EnvSensing/UsMpu6050/UsMpu6050ComponentAc.hpp"
#include "Components/EnvSensing/UsMpu6050/mpu6050_core.h"

namespace Components {

class UsMpu6050 final : public UsMpu6050ComponentBase {
 public:
  explicit UsMpu6050(const char* compName);
  ~UsMpu6050() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void emitSample(const MpuSampleOut& s);

  static bool busWrite(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
  static bool busRead(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len);

  Mpu6050Core* m_core;
  U32 m_tlmDiv;
  ImuDrvState m_lastState;
};

}  // namespace Components
#endif
