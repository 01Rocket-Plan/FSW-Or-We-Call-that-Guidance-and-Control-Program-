// ======================================================================
// \title  Lsm6dsoDriver.hpp
// \author 環境感測組 Farhad（LSM6DSO 備援分支；外殼沿用 main ImuDriver 寫法）
// \brief  ST LSM6DSO — F´ 外殼；邏輯在 lsm6dso_core（跟 ImuDriver 同埠／遙測／事件，可互換）
// Base ID 0x4000（沿用 IMU 槽位）；Sim 0x4080
// ======================================================================
#ifndef COMPONENTS_LSM6DSODRIVER_HPP
#define COMPONENTS_LSM6DSODRIVER_HPP

#include "Components/EnvSensing/Lsm6dsoDriver/Lsm6dsoDriverComponentAc.hpp"
#include "Components/EnvSensing/Lsm6dsoDriver/lsm6dso_core.h"

namespace Components {

class Lsm6dsoDriver final : public Lsm6dsoDriverComponentBase {
 public:
  explicit Lsm6dsoDriver(const char* compName);
  ~Lsm6dsoDriver() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void emitSample(const LsmSampleOut& s);
  void writeTlm();

  static bool busWrite(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
  static bool busRead(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len);

  LsmCore* m_core;
  U32 m_tlmDiv;
  LsmDrvState m_lastState;
  bool m_gyroSatLatched;   // 飽和事件只在剛開始飽和那一拍記
  bool m_accelSatLatched;
};

}  // namespace Components
#endif
