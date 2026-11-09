// ======================================================================
// \title  ActuatorMgr.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  F´ 外殼；邏輯在 ActuatorMgrCore。Base ID 0x1300。
// ======================================================================
#ifndef COMPONENTS_MCU_ACTUATORMGR_ACTUATORMGR_HPP
#define COMPONENTS_MCU_ACTUATORMGR_ACTUATORMGR_HPP

#include "Components/MCU/ActuatorMgr/ActuatorMgrComponentAc.hpp"
#include "Components/MCU/ActuatorMgr/ActuatorMgrCore.hpp"

namespace Mcu {

class ActuatorMgr final : public ActuatorMgrComponentBase {
 public:
  explicit ActuatorMgr(const char* compName);
  ~ActuatorMgr() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void finCmdIn_handler(FwIndexType portNum, const RocketTypes::FinCmd& c) override;
  void sepCmdIn_handler(FwIndexType portNum, const RocketTypes::SepCmd& c) override;
  void uvIn_handler(FwIndexType portNum, const RocketTypes::UvState& st) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  F64 nowSeconds();

  ActuatorMgrCore m_core;
  RocketTypes::UvState m_uv;
  U32 m_tlmDiv;
  bool m_lastLocked;
  bool m_firstStep;
};

}  // namespace Mcu

#endif
