// ======================================================================
// \title  FlightMode.hpp
// \author 殼：MCU 組（EdwardWang+Claude）｜邏輯：Edward 本人
// \brief  F´ 外殼；邏輯在 FlightModeCore（目前是殼）。Base ID 0x1400。
// ======================================================================
#ifndef COMPONENTS_MCU_FLIGHTMODE_FLIGHTMODE_HPP
#define COMPONENTS_MCU_FLIGHTMODE_FLIGHTMODE_HPP

#include "Components/MCU/FlightMode/FlightModeComponentAc.hpp"
#include "Components/MCU/FlightMode/FlightModeCore.hpp"

namespace Mcu {

class FlightMode final : public FlightModeComponentBase {
 public:
  explicit FlightMode(const char* compName);
  ~FlightMode() override;

  //! 拓樸設定階段呼叫（DsMcu → DS、UsMcu → US）。prmDb 裡有 role 時以 prmDb 為準。
  void setDeploymentRole(FmRole role);

 private:
  void applyRole(FmRole role);
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void stateIn_handler(FwIndexType portNum, const RocketTypes::StateOut& s) override;
  void uvIn_handler(FwIndexType portNum, const RocketTypes::UvState& st) override;
  void discreteIn_handler(FwIndexType portNum, const RocketTypes::DiscreteSample& s) override;
  void stageLinkIn_handler(FwIndexType portNum, const RocketTypes::StageLinkMsg& m) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  F64 nowSeconds();
  void emitOutputs(const FmOutputs& o);

  FlightModeCore m_core;
  FmRole m_deployRole;
  FmInputs m_in;
  FmGroundCmd m_cmd;
  RocketTypes::FlightModeId m_lastMode;
  U32 m_sepSeq;
  U32 m_chuteSeq;
  U32 m_linkSeq;
  U32 m_tlmDiv;
  bool m_warnedShell;
};

}  // namespace Mcu

#endif
