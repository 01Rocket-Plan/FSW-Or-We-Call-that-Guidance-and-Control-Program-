// ======================================================================
// \title  Controller.cpp
// \author MCU 組（EdwardWang+Claude）
// \brief  F´ 外殼：搬 StateOut／模式進 core、1 kHz step、送 FinCmd、10 Hz 遙測。
// ======================================================================
#include "Components/MCU/Controller/Controller.hpp"

#include <cmath>

namespace Mcu {

namespace {
constexpr U32 kTlmDivider = 100;  // 1 kHz → 10 Hz
}

Controller::Controller(const char* compName)
    : ControllerComponentBase(compName), m_tlmDiv(0), m_satCount(0), m_lastClosed(false) {}

Controller::~Controller() {}

bool Controller::modeAllowsClosedLoop(const RocketTypes::FlightModeId& mode) {
  // SSS flight_mode.py _authority：BOOST_ACTIVE／COAST 才 CLOSED_LOOP。
  // 中止（ABORT）、分離後、被動助推、墊上、掛火 → 一律不閉環。
  return mode == RocketTypes::FlightModeId::BOOST_ACTIVE || mode == RocketTypes::FlightModeId::COAST;
}

F64 Controller::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void Controller::loadParams() {
  Fw::ParamValid v;
  ControllerParams p;
  p.kp = this->paramGet_kp(v);
  p.kd = this->paramGet_kd(v);
  p.finMax_rad = this->paramGet_finMax_rad(v);
  p.stateStale_s = this->paramGet_stateStale_s(v);
  p.demo50mFixed = this->paramGet_demo50mFixed(v);
  m_core.setParams(p);
}

void Controller::parameterUpdated(FwPrmIdType) { loadParams(); }
void Controller::parametersLoaded() { loadParams(); }

void Controller::stateIn_handler(FwIndexType, const RocketTypes::StateOut& s) {
  StateIn in;
  in.t_s = s.get_t_s();
  in.seq = s.get_seq();
  for (U32 i = 0; i < 4; ++i) in.q_wxyz[i] = s.get_q_wxyz()[i];
  for (U32 i = 0; i < 3; ++i) in.rate_body_rads[i] = s.get_rate_body_rads()[i];
  in.imuValid = s.get_imuValid();
  m_core.pushState(in);
}

void Controller::modeIn_handler(FwIndexType, const RocketTypes::FlightModeId& mode) {
  m_core.setMode(modeAllowsClosedLoop(mode));
}

void Controller::schedIn_handler(FwIndexType, U32) {
  FinOut o;
  m_core.step(this->nowSeconds(), o);

  const bool closed = !o.lockNeutral;
  const CtlHoldReason reason(static_cast<CtlHoldReason::T>(static_cast<U8>(o.reason)));
  if (closed && !m_lastClosed) this->log_ACTIVITY_HI_ClosedLoopOn();
  if (!closed && m_lastClosed) this->log_ACTIVITY_HI_ClosedLoopOff(reason);
  m_lastClosed = closed;
  if (closed) {
    const F32 lim = m_core.params().finMax_rad;
    if (std::fabs(o.fin_rad[0]) >= lim || std::fabs(o.fin_rad[1]) >= lim) ++m_satCount;
  }

  if (this->isConnected_finCmdOut_OutputPort(0)) {
    const RocketTypes::FinCmd::Type_of_fin_rad fins = {o.fin_rad[0], o.fin_rad[1], o.fin_rad[2], o.fin_rad[3]};
    this->finCmdOut_out(0, RocketTypes::FinCmd(o.t_s, o.seq, fins, o.lockNeutral));
  }

  if (++m_tlmDiv >= kTlmDivider) {
    m_tlmDiv = 0;
    this->tlmWrite_closedLoop(closed);
    this->tlmWrite_holdReason(reason);
    this->tlmWrite_errX_rad(o.err_rad[0]);
    this->tlmWrite_errY_rad(o.err_rad[1]);
    this->tlmWrite_fin1_rad(o.fin_rad[0]);
    this->tlmWrite_fin2_rad(o.fin_rad[1]);
    this->tlmWrite_fin3_rad(o.fin_rad[2]);
    this->tlmWrite_fin4_rad(o.fin_rad[3]);
    this->tlmWrite_satCount(m_satCount);
  }
}

}  // namespace Mcu
