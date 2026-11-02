#include "Components/Servo/SepServoDriverSim/SepServoDriverSim.hpp"

namespace Components {

namespace {
// 與真驅動 SepServoDriver.cpp 相同的原因碼
enum DropReason : U8 { DROP_NOT_READY = 1, DROP_SEQ = 2, DROP_INVALID = 3 };
enum FaultReason : U8 { FAULT_SELFTEST = 1 };
constexpr U32 kPeriodTicks = 20;  // rg1kHz → 50 Hz 推進核心
}  // namespace

SepServoDriverSim::SepServoDriverSim(const char* compName)
    : SepServoDriverSimComponentBase(compName),
      m_core(ServoSim::SepParams(), *this, *this, *this),
      m_out_us{1000, 1000, 1000, 1000},
      m_tick(0),
      m_started(false) {}

void SepServoDriverSim::parametersLoaded() {
  Fw::ParamValid v;
  ServoSim::SepParams p;
  p.lock_us = this->paramGet_lock_us(v);
  p.fire_us = this->paramGet_fire_us(v);
  p.fire_hold_s = this->paramGet_fire_hold_s(v);
  p.pulse_min_us = this->paramGet_pulse_min_us(v);
  p.pulse_max_us = this->paramGet_pulse_max_us(v);
  p.selftest_deadline_s = this->paramGet_selftest_deadline_s(v);
  p.tick_hz = 1000.0f / static_cast<F32>(kPeriodTicks);
  m_core.configure(p);
}

F64 SepServoDriverSim::now_s() {
  const Fw::Time t = this->getTime();
  return static_cast<F64>(t.getSeconds()) + static_cast<F64>(t.getUSeconds()) * 1e-6;
}

void SepServoDriverSim::schedIn_handler(FwIndexType, U32) {
  if (!m_started) { m_core.start(); m_started = true; this->tlmWrite_drvState(RocketTypes::DrvState::INIT); }
  if (++m_tick < kPeriodTicks) return;
  m_tick = 0;
  m_core.tick();
  // 與真驅動相同：pwmOut 的 channelMask = 已釋放（含保持中）通道
  if (this->isConnected_pwmOut_OutputPort(0)) {
    const U8 mask = static_cast<U8>(m_core.activeMask() | m_core.firedMask());
    this->pwmOut_out(0, RocketTypes::SepCmd(now_s(), m_core.lastSeq(), mask, true));
  }
  this->tlmWrite_drvState(static_cast<RocketTypes::DrvState::T>(m_core.drvState()));
  this->tlmWrite_lastSeq(m_core.lastSeq());
  this->tlmWrite_firedMask(m_core.firedMask());
  this->tlmWrite_lockedMask(m_core.lockedMask());
  this->tlmWrite_uvState(static_cast<RocketTypes::UvState::T>(m_core.uvState()));
}

void SepServoDriverSim::cmdIn_handler(FwIndexType, const RocketTypes::SepCmd& c) {
  ServoSim::SepCmd k{c.get_t_s(), c.get_seq(), c.get_channelMask(), c.get_valid()};
  m_core.cmdIn(k);
}

void SepServoDriverSim::uvIn_handler(FwIndexType, const RocketTypes::UvState& st) {
  m_core.uvIn(static_cast<ServoSim::UvState>(st.e));
}

bool SepServoDriverSim::sepReleasedGet(uint8_t& mask) {
  if (!this->isConnected_sepReleasedGet_OutputPort(0)) { mask = 0; return true; }  // 同真驅動：未接視為全未釋放
  mask = this->sepReleasedGet_out(0);
  return true;
}

bool SepServoDriverSim::sepReleasedSet(uint8_t mask) {
  if (!this->isConnected_sepReleasedSet_OutputPort(0)) return false;  // 未接：不能保證只釋放一次 → 不釋放
  return this->sepReleasedSet_out(0, mask) == Fw::Success::SUCCESS;
}

void SepServoDriverSim::pwmOut(uint8_t ch, uint16_t used_us) { if (ch < 4) m_out_us[ch] = used_us; }

void SepServoDriverSim::evEnterReady() { this->log_ACTIVITY_HI_EnterReady(); }
void SepServoDriverSim::evEnterFault(ServoSim::FaultReason) { this->log_WARNING_HI_EnterFault(FAULT_SELFTEST); }
void SepServoDriverSim::evCmdDropped(uint32_t seq, ServoSim::DropReason r) {
  const U8 code = r == ServoSim::DropReason::NOT_READY ? DROP_NOT_READY : (r == ServoSim::DropReason::INVALID ? DROP_INVALID : DROP_SEQ);
  this->log_WARNING_LO_CmdDropped(seq, code);
}
void SepServoDriverSim::evSepFire(uint8_t ch) { this->log_ACTIVITY_HI_SepFire(ch); }
void SepServoDriverSim::evSepAlreadyFired(uint8_t ch) { this->log_WARNING_LO_SepAlreadyFired(ch); }
void SepServoDriverSim::evSepFaultHold(uint8_t mask) { this->log_WARNING_HI_SepFaultHold(mask); }
void SepServoDriverSim::evSepNvWriteFail(uint8_t ch) { this->log_WARNING_HI_SepNvWriteFail(ch); }
void SepServoDriverSim::evUvSeen(ServoSim::UvState st) { this->log_ACTIVITY_HI_UvSeen(static_cast<RocketTypes::UvState::T>(st)); }

}  // namespace Components
