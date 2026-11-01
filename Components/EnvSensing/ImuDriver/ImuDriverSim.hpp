// ======================================================================
// \title  ImuDriverSim.hpp
// \brief  ImuDriver Sim — Base ID 0x4080
// ======================================================================
#ifndef COMPONENTS_IMUDRIVERSIM_HPP
#define COMPONENTS_IMUDRIVERSIM_HPP

#include "Components/EnvSensing/ImuDriver/ImuDriverSimComponentAc.hpp"
#include "Components/EnvSensing/ImuDriver/ImuSimDevice.hpp"
#include "Components/EnvSensing/ImuDriver/imu_core.h"

namespace Components {

class ImuDriverSim final : public ImuDriverSimComponentBase {
 public:
  explicit ImuDriverSim(const char* compName);
  ~ImuDriverSim() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void injectFault_handler(FwIndexType portNum, U8 code) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void emitSample(const ImuSampleOut& s);

  ImuSimDevice m_dev;
  ImuCore* m_core;
  U32 m_tlmDiv;
};

}  // namespace Components
#endif
