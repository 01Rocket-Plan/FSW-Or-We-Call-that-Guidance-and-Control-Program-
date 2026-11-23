// ======================================================================
// \title  Lsm6dsoDriverSim.hpp
// \brief  Lsm6dsoDriver Sim — Base ID 0x4080（晶片暫存器級模擬＋同一顆 lsm6dso_core）
// ======================================================================
#ifndef COMPONENTS_LSM6DSODRIVERSIM_HPP
#define COMPONENTS_LSM6DSODRIVERSIM_HPP

#include "Components/EnvSensing/Lsm6dsoDriver/Lsm6dsoDriverSimComponentAc.hpp"
#include "Components/EnvSensing/Lsm6dsoDriver/Lsm6dsoSimDevice.hpp"
#include "Components/EnvSensing/Lsm6dsoDriver/lsm6dso_core.h"

namespace Components {

class Lsm6dsoDriverSim final : public Lsm6dsoDriverSimComponentBase {
 public:
  explicit Lsm6dsoDriverSim(const char* compName);
  ~Lsm6dsoDriverSim() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void injectFault_handler(FwIndexType portNum, U8 code) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void emitSample(const LsmSampleOut& s);

  Lsm6dsoSimDevice m_dev;
  LsmCore* m_core;
  U32 m_tlmDiv;
  LsmDrvState m_lastState;
};

}  // namespace Components
#endif
