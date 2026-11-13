// ======================================================================
// \title  ImuDriver.hpp
// \author 環境感測系統組子Bot1（2026-11-10 EdwardWang+Claude：改單顆、機體系 DCM）
// \brief  ICM-20948 — F´ 外殼；邏輯在 imu_core
// Base ID 0x4000（拓樸設定）；Sim 0x4080
// ======================================================================
#ifndef COMPONENTS_IMUDRIVER_HPP
#define COMPONENTS_IMUDRIVER_HPP

#include "Components/EnvSensing/ImuDriver/ImuDriverComponentAc.hpp"
#include "Components/EnvSensing/ImuDriver/imu_core.h"

namespace Components {

class ImuDriver final : public ImuDriverComponentBase {
 public:
  explicit ImuDriver(const char* compName);
  ~ImuDriver() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void emitSample(const ImuSampleOut& s);
  void writeTlm();

  static bool busWrite(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
  static bool busRead(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len);

  ImuCore* m_core;
  U32 m_tlmDiv;
  ImuDrvState m_lastState;
  bool m_gyroSatLatched;   // 飽和事件只在剛開始飽和那一拍記
  bool m_accelSatLatched;
};

}  // namespace Components
#endif
