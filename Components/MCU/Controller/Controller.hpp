// ======================================================================
// \title  Controller.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  F´ 外殼；邏輯在 ControllerCore。Base ID 0x1200。
// ======================================================================
#ifndef COMPONENTS_MCU_CONTROLLER_CONTROLLER_HPP
#define COMPONENTS_MCU_CONTROLLER_CONTROLLER_HPP

#include "Components/MCU/Controller/ControllerComponentAc.hpp"
#include "Components/MCU/Controller/ControllerCore.hpp"

namespace Mcu {

class Controller final : public ControllerComponentBase {
 public:
  explicit Controller(const char* compName);
  ~Controller() override;

  //! 哪些模式可以閉環（SSS _authority：BOOST_ACTIVE、COAST）。公開給測試與審查。
  static bool modeAllowsClosedLoop(const RocketTypes::FlightModeId& mode);

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void stateIn_handler(FwIndexType portNum, const RocketTypes::StateOut& s) override;
  void modeIn_handler(FwIndexType portNum, const RocketTypes::FlightModeId& mode) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  F64 nowSeconds();

  ControllerCore m_core;
  U32 m_tlmDiv;
  U32 m_satCount;
  bool m_lastClosed;
};

}  // namespace Mcu

#endif
