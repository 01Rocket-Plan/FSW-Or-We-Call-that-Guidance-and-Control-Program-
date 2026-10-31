#include "Components/EnvSensing/BaroDriver/BaroDriverSim.hpp"
#include <cmath>

namespace Components {

static RocketTypes::DrvState toFppS(DrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

BaroDriverSim::BaroDriverSim(const char* compName)
    : BaroDriverSimComponentBase(compName),
      m_core(m_dev, *this),
      m_simP_Pa(101325.0),
      m_simT_C(25.0),
      m_simH_pct(45.0),
      m_simG_ohm(50000.0),
      m_tlmDiv(0) {
  m_dev.setTruth(m_simP_Pa, m_simT_C, m_simH_pct, m_simG_ohm);
}

void BaroDriverSim::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  m_dev.advance();
  m_core.tick(t_s);
  if (++m_tlmDiv >= 100) {
    m_tlmDiv = 0;
    this->tlmWrite_HsState(toFppS(m_core.state()));
    this->tlmWrite_ErrorCount(m_core.errorCount());
    this->tlmWrite_RefPress_Pa(m_core.p0_Pa());
    this->tlmWrite_BaroSeq(m_core.baroSeq());
  }
}

void BaroDriverSim::SIM_SET_ALT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 alt_m) {
  m_simP_Pa = 101325.0 * std::pow(1.0 - static_cast<F64>(alt_m) / 44330.0, 5.255);
  m_dev.setTruth(m_simP_Pa, m_simT_C, m_simH_pct, m_simG_ohm);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void BaroDriverSim::SIM_SET_ENV_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 temp_C, F32 humidity_pct, F32 gas_ohm) {
  m_simT_C = temp_C;
  m_simH_pct = humidity_pct;
  m_simG_ohm = gas_ohm;
  m_dev.setTruth(m_simP_Pa, m_simT_C, m_simH_pct, m_simG_ohm);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void BaroDriverSim::SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Components::BaroSimFault& fault) {
  m_dev.setFault(static_cast<Bme680SimFault>(fault.e));
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void BaroDriverSim::onBaro(const BaroOut& s) {
  if (!this->isConnected_baroOut_OutputPort(0)) return;
  this->baroOut_out(0, RocketTypes::BaroSample(RocketTypes::SampleHeader(s.t_s, s.seq, s.valid), s.press_Pa, s.temp_C, s.alt_m));
}

void BaroDriverSim::onEnv(const EnvOut& s) {
  if (!this->isConnected_envOut_OutputPort(0)) return;
  this->envOut_out(0, RocketTypes::EnvSample(RocketTypes::SampleHeader(s.t_s, s.seq, s.valid), s.humidity_pct, s.gas_ohm));
}

void BaroDriverSim::onStateChange(DrvState from, DrvState to) {
  this->log_ACTIVITY_HI_StateChanged(toFppS(from), toFppS(to));
  this->tlmWrite_HsState(toFppS(to));
}

void BaroDriverSim::onFault(BaroFault code, uint32_t detail) {
  this->log_WARNING_HI_Fault(static_cast<U8>(code), detail);
}

}  // namespace Components
