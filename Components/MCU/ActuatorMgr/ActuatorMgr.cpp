// ======================================================================
// \title  ActuatorMgr.cpp
// \author MCU 組（EdwardWang+Claude）
// \brief  F´ 外殼：收 FinCmd／SepCmd／UvState、1 kHz step、送 ServoCmd／SepCmd、10 Hz 遙測。
// ======================================================================
#include "Components/MCU/ActuatorMgr/ActuatorMgr.hpp"

namespace Mcu {

namespace {
constexpr U32 kTlmDivider = 100;  // 1 kHz → 10 Hz
}

ActuatorMgr::ActuatorMgr(const char* compName)
    : ActuatorMgrComponentBase(compName),
      m_uv(RocketTypes::UvState::NOMINAL),
      m_tlmDiv(0),
      m_lastLocked(true),
      m_firstStep(true) {}

ActuatorMgr::~ActuatorMgr() {}

F64 ActuatorMgr::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void ActuatorMgr::loadParams() {
  Fw::ParamValid v;
  ActuatorMgrParams p;
  p.neutral_us = this->paramGet_neutral_us(v);
  p.pulseMin_us = this->paramGet_pulseMin_us(v);
  p.pulseMax_us = this->paramGet_pulseMax_us(v);
  p.usPerRad = this->paramGet_usPerRad(v);
  p.gear = this->paramGet_gear(v);
  p.dir[0] = this->paramGet_dir1(v);
  p.dir[1] = this->paramGet_dir2(v);
  p.dir[2] = this->paramGet_dir3(v);
  p.dir[3] = this->paramGet_dir4(v);
  p.trim_rad[0] = this->paramGet_trim1_rad(v);
  p.trim_rad[1] = this->paramGet_trim2_rad(v);
  p.trim_rad[2] = this->paramGet_trim3_rad(v);
  p.trim_rad[3] = this->paramGet_trim4_rad(v);
  p.finMax_rad = this->paramGet_finMax_rad(v);
  p.rateMax_rads = this->paramGet_rateMax_rads(v);
  p.finStale_s = this->paramGet_finStale_s(v);
  p.uvLockLevel = static_cast<uint8_t>(this->paramGet_uvLockLevel(v).e);
  p.sepStagger_s = this->paramGet_sepStagger_s(v);
  m_core.setParams(p);
}

void ActuatorMgr::parameterUpdated(FwPrmIdType) { loadParams(); }
void ActuatorMgr::parametersLoaded() { loadParams(); }

void ActuatorMgr::finCmdIn_handler(FwIndexType, const RocketTypes::FinCmd& c) {
  FinIn in;
  in.t_s = c.get_t_s();
  in.seq = c.get_seq();
  for (U32 i = 0; i < 4; ++i) in.fin_rad[i] = c.get_fin_rad()[i];
  in.lockNeutral = c.get_lockNeutral();
  m_core.pushFin(in);
}

void ActuatorMgr::sepCmdIn_handler(FwIndexType, const RocketTypes::SepCmd& c) {
  SepIn in;
  in.t_s = c.get_t_s();
  in.seq = c.get_seq();
  in.channelMask = c.get_channelMask();
  in.valid = c.get_valid();
  if (!m_core.pushSep(in, this->nowSeconds())) this->log_WARNING_LO_SepDropped(in.seq);
}

void ActuatorMgr::uvIn_handler(FwIndexType, const RocketTypes::UvState& st) {
  m_uv = st;
  m_core.setUv(static_cast<uint8_t>(st.e));
}

void ActuatorMgr::schedIn_handler(FwIndexType, U32) {
  ServoOut o;
  SepOut s;
  m_core.step(this->nowSeconds(), o, s);

  const ActLockReason reason(static_cast<ActLockReason::T>(static_cast<U8>(o.reason)));
  if (o.locked && (!m_lastLocked || m_firstStep)) this->log_ACTIVITY_HI_Locked(reason);
  if (!o.locked && m_lastLocked) this->log_ACTIVITY_HI_Unlocked();
  m_lastLocked = o.locked;
  m_firstStep = false;

  if (this->isConnected_servoCmdOut_OutputPort(0)) {
    const RocketTypes::ServoCmd::Type_of_pulse_us p = {o.pulse_us[0], o.pulse_us[1], o.pulse_us[2], o.pulse_us[3]};
    this->servoCmdOut_out(0, RocketTypes::ServoCmd(o.t_s, o.seq, p, o.lockNeutral));
  }
  if (s.send) {
    this->log_ACTIVITY_HI_SepForwarded(s.channelMask);
    if (this->isConnected_sepCmdOut_OutputPort(0)) {
      this->sepCmdOut_out(0, RocketTypes::SepCmd(s.t_s, s.seq, s.channelMask, true));
    }
  }

  if (++m_tlmDiv >= kTlmDivider) {
    m_tlmDiv = 0;
    this->tlmWrite_locked(o.locked);
    this->tlmWrite_lockReason(reason);
    this->tlmWrite_pulse1_us(o.pulse_us[0]);
    this->tlmWrite_pulse2_us(o.pulse_us[1]);
    this->tlmWrite_pulse3_us(o.pulse_us[2]);
    this->tlmWrite_pulse4_us(o.pulse_us[3]);
    this->tlmWrite_clampCount(o.clampCount);
    this->tlmWrite_rateLimitCount(o.rateLimitCount);
    this->tlmWrite_uvState(m_uv);
    this->tlmWrite_sepPendingMask(m_core.pendingSepMask());
    this->tlmWrite_sepDropCount(m_core.sepDropCount());
  }
}

}  // namespace Mcu
