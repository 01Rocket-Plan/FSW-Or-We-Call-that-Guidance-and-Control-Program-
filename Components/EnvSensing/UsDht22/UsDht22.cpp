#include "Components/EnvSensing/UsDht22/UsDht22.hpp"

namespace Components {

static RocketTypes::DrvState toFpp(DhtState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

UsDht22::UsDht22(const char* compName) : UsDht22ComponentBase(compName), m_io(nullptr), m_core(*this, *this) {}

void UsDht22::configure(IDht22Io& io) { m_io = &io; }

void UsDht22::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  m_core.tick(t_s);
  this->tlmWrite_HsState(toFpp(m_core.state()));
  this->tlmWrite_ErrorCount(m_core.errorCount());
  this->tlmWrite_Seq(m_core.seq());
}

bool UsDht22::configure() { return m_io != nullptr && m_io->configure(); }
bool UsDht22::start() { return m_io != nullptr && m_io->start(); }
int UsDht22::fetchHighs(uint16_t* h, int max) { return m_io != nullptr ? m_io->fetchHighs(h, max) : -1; }

void UsDht22::onSample(const HumTempOut& s) {
  if (!this->isConnected_humTempOut_OutputPort(0)) return;
  this->humTempOut_out(0, RocketTypes::HumTempSample(RocketTypes::SampleHeader(s.t_s, s.seq, s.valid), s.temp_K, s.humidity_frac));
}

void UsDht22::onStateChange(DhtState from, DhtState to) { this->log_ACTIVITY_HI_StateChanged(toFpp(from), toFpp(to)); }

void UsDht22::onFault(DhtFault code, uint32_t detail) { this->log_WARNING_HI_Fault(static_cast<U8>(code), detail); }

}  // namespace Components
