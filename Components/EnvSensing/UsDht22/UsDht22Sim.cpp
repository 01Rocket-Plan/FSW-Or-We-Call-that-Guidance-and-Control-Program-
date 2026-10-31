#include "Components/EnvSensing/UsDht22/UsDht22Sim.hpp"

namespace EnvSensing {

static RocketTypes::DrvState toFppS(DhtState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

UsDht22Sim::UsDht22Sim(const char* compName) : UsDht22SimComponentBase(compName), m_core(m_io, *this) {}

void UsDht22Sim::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  m_core.tick(t_s);
  this->tlmWrite_HsState(toFppS(m_core.state()));
  this->tlmWrite_ErrorCount(m_core.errorCount());
  this->tlmWrite_Seq(m_core.seq());
}

void UsDht22Sim::SIM_SET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 temp_C, F32 humidity_pct) {
  m_io.setTruth(temp_C, humidity_pct);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void UsDht22Sim::SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const EnvSensing::DhtSimFault& fault) {
  m_io.setFault(static_cast<Dht22SimFault>(fault.e));
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void UsDht22Sim::onSample(const HumTempOut& s) {
  if (!this->isConnected_humTempOut_OutputPort(0)) return;
  this->humTempOut_out(0, RocketTypes::HumTempSample(RocketTypes::SampleHeader(s.t_s, s.seq, s.valid), s.temp_K, s.humidity_frac));
}

void UsDht22Sim::onStateChange(DhtState from, DhtState to) { this->log_ACTIVITY_HI_StateChanged(toFppS(from), toFppS(to)); }

void UsDht22Sim::onFault(DhtFault code, uint32_t detail) { this->log_WARNING_HI_Fault(static_cast<U8>(code), detail); }

}  // namespace EnvSensing
