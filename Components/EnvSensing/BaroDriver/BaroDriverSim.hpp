// BaroDriverSim：同一個 Bme680Core 接暫存器層模擬器
#ifndef ENVSENSING_BARODRIVERSIM_HPP
#define ENVSENSING_BARODRIVERSIM_HPP

#include "Components/EnvSensing/BaroDriver/BaroDriverSimComponentAc.hpp"
#include "Components/EnvSensing/BaroDriver/Bme680Core.hpp"
#include "Components/EnvSensing/BaroDriver/Bme680SimDevice.hpp"

namespace EnvSensing {

class BaroDriverSim final : public BaroDriverSimComponentBase, private IBme680Sink {
 public:
  explicit BaroDriverSim(const char* compName);
  ~BaroDriverSim() override = default;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void SIM_SET_ALT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 alt_m) override;
  void SIM_SET_ENV_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 temp_C, F32 humidity_pct, F32 gas_ohm) override;
  void SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const EnvSensing::BaroSimFault& fault) override;

  void onBaro(const BaroOut& s) override;
  void onEnv(const EnvOut& s) override;
  void onStateChange(DrvState from, DrvState to) override;
  void onFault(BaroFault code, uint32_t detail) override;

  Bme680SimDevice m_dev;
  Bme680Core m_core;
  F64 m_simP_Pa;
  F64 m_simT_C;
  F64 m_simH_pct;
  F64 m_simG_ohm;
  U32 m_tlmDiv;
};

}  // namespace EnvSensing
#endif
