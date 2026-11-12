// ======================================================================
// \title  Estimator.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  F´ 外殼；邏輯在 EstimatorCore。Base ID 0x1100。
// ======================================================================
#ifndef COMPONENTS_MCU_ESTIMATOR_ESTIMATOR_HPP
#define COMPONENTS_MCU_ESTIMATOR_ESTIMATOR_HPP

#include "Components/MCU/Estimator/EstimatorComponentAc.hpp"
#include "Components/MCU/Estimator/EstimatorCore.hpp"

namespace Mcu {

class Estimator final : public EstimatorComponentBase {
 public:
  explicit Estimator(const char* compName);
  ~Estimator() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void imuIn_handler(FwIndexType portNum, const RocketTypes::ImuSample& s) override;
  void baroIn_handler(FwIndexType portNum, const RocketTypes::BaroSample& s) override;
  void modeIn_handler(FwIndexType portNum, const RocketTypes::FlightModeId& mode) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void writeTlm(const StateOutC& o);
  F64 nowSeconds();

  EstimatorCore m_core;
  U32 m_tlmDiv;
  bool m_lastAligned;
  bool m_lastFresh;
  bool m_lastOnPad;
};

}  // namespace Mcu

#endif
