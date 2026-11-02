#include "Components/Servo/ServoDriverSim/ServoDriverSim.hpp"

namespace Components {

namespace {
// 與真驅動 ServoDriver.cpp 相同的原因碼
enum DropReason : U8 { DROP_NOT_READY = 1, DROP_SEQ = 2 };
enum FaultReason : U8 { FAULT_SELFTEST = 1, FAULT_PWM = 2, FAULT_MODEL_ROLE = 3 };
enum LockSource : U8 { LOCK_CMD = 1, LOCK_TIMEOUT = 2 };
}  // namespace

ServoDriverSim::ServoDriverSim(const char* compName)
    : ServoDriverSimComponentBase(compName),
      m_core(ServoSim::ServoParams(), *this, *this),
      m_out_us{1500, 1500, 1500, 1500},
      m_tick(0),
      m_periodTicks(20),
      m_started(false),
      m_lockNeutral(false),
      m_model(Components::ServoModel::SG90),
      m_role(Components::ServoRole::TAIL) {}

void ServoDriverSim::parametersLoaded() {
  Fw::ParamValid v;
  ServoSim::ServoParams p;
  m_model = this->paramGet_model(v);
  m_role = this->paramGet_role(v);
  p.model = static_cast<ServoSim::ServoModel>(static_cast<U8>(m_model.e));
  p.role = static_cast<ServoSim::ServoRole>(static_cast<U8>(m_role.e));
  p.neutral_us = this->paramGet_neutral_us(v);
  p.pulse_min_us = this->paramGet_pulse_min_us(v);
  p.pulse_max_us = this->paramGet_pulse_max_us(v);
  p.cmd_timeout_s = this->paramGet_cmd_timeout_s(v);
  const U16 hz = this->paramGet_pwm_hz(v);
  p.pwm_hz = static_cast<F32>(hz ? hz : 50);
  p.channel_count = this->paramGet_channel_count(v);
  if (p.channel_count > 4) p.channel_count = 4;
  p.selftest_deadline_s = this->paramGet_selftest_deadline_s(v);
  m_core.configure(p);
  m_periodTicks = hz ? (1000u / hz) : 20u;  // rg1kHz → pwm period ticks
  if (m_periodTicks == 0) m_periodTicks = 1;
}

F64 ServoDriverSim::now_s() {
  const Fw::Time t = this->getTime();
  return static_cast<F64>(t.getSeconds()) + static_cast<F64>(t.getUSeconds()) * 1e-6;
}

void ServoDriverSim::schedIn_handler(FwIndexType, U32) {
  if (!m_started) { m_core.start(); m_started = true; this->tlmWrite_drvState(RocketTypes::DrvState::INIT); }
  if (++m_tick < m_periodTicks) return;  // 50 Hz 零階保持：每個 PWM 週期推進核心一次
  m_tick = 0;
  m_core.tick();
  // Advance SG90 shaft-angle sim by one PWM period (50 Hz → 0.02 s).
  // Angle available via m_sg90Dev.angleDeg(ch) for future SIL telemetry.
  m_sg90Dev.tick(static_cast<F32>(m_periodTicks) / 1000.0f);  // rg1kHz → PWM period
  U16 pulses[4] = {m_out_us[0], m_out_us[1], m_out_us[2], m_out_us[3]};
  if (this->isConnected_pwmOut_OutputPort(0)) {
    this->pwmOut_out(0, RocketTypes::ServoCmd(now_s(), m_core.lastSeq(), RocketTypes::ServoCmd::Type_of_pulse_us(pulses), m_lockNeutral));
  }
  writeTlm();
}

void ServoDriverSim::cmdIn_handler(FwIndexType, const RocketTypes::ServoCmd& c) {
  ServoSim::ServoCmd k{c.get_t_s(), c.get_seq(), {c.get_pulse_us()[0], c.get_pulse_us()[1], c.get_pulse_us()[2], c.get_pulse_us()[3]}, c.get_lockNeutral()};
  m_lockNeutral = c.get_lockNeutral();
  m_core.cmdIn(k);
}

void ServoDriverSim::uvIn_handler(FwIndexType, const RocketTypes::UvState& st) {
  m_core.uvIn(static_cast<ServoSim::UvState>(st.e));
}

void ServoDriverSim::pwmOut(uint8_t ch, uint16_t used_us) {
  if (ch < 4) {
    m_out_us[ch] = used_us;
    m_sg90Dev.setCommandPulseUs(ch, used_us);  // µs → slew sim
  }
}

void ServoDriverSim::writeTlm() {
  this->tlmWrite_drvState(static_cast<RocketTypes::DrvState::T>(m_core.drvState()));
  this->tlmWrite_lastSeq(m_core.lastSeq());
  this->tlmWrite_lastPulse0_us(m_out_us[0]);
  this->tlmWrite_lastPulse1_us(m_out_us[1]);
  this->tlmWrite_lastPulse2_us(m_out_us[2]);
  this->tlmWrite_lastPulse3_us(m_out_us[3]);
  this->tlmWrite_cmdAge_s(m_core.cmdAge_s());
  this->tlmWrite_uvState(static_cast<RocketTypes::UvState::T>(m_core.uvState()));
  this->tlmWrite_clampCount(m_core.clampCount());
  this->tlmWrite_dropCount(m_core.dropCount());
  this->tlmWrite_model(m_model);
  this->tlmWrite_role(m_role);
}

void ServoDriverSim::evEnterReady() { this->log_ACTIVITY_HI_EnterReady(); }
void ServoDriverSim::evEnterFault(ServoSim::FaultReason r) {
  const U8 code = (r == ServoSim::FaultReason::MODEL_ROLE_MISMATCH) ? FAULT_MODEL_ROLE : FAULT_SELFTEST;
  this->log_WARNING_HI_EnterFault(code);
}
void ServoDriverSim::evCmdDropped(uint32_t seq, ServoSim::DropReason r) {
  this->log_WARNING_LO_CmdDropped(seq, r == ServoSim::DropReason::NOT_READY ? DROP_NOT_READY : DROP_SEQ);
}
void ServoDriverSim::evPulseClamped(uint8_t ch, uint16_t raw, uint16_t used) { this->log_WARNING_LO_PulseClamped(ch, raw, used); }
void ServoDriverSim::evCmdTimeout(float age_s) { m_lockNeutral = false; this->log_WARNING_HI_CmdTimeout(age_s); }
void ServoDriverSim::evLockNeutral(ServoSim::NeutralSource s) {
  this->log_ACTIVITY_HI_LockNeutral(s == ServoSim::NeutralSource::CMD ? LOCK_CMD : LOCK_TIMEOUT);
}
void ServoDriverSim::evUvSeen(ServoSim::UvState st) { this->log_ACTIVITY_HI_UvSeen(static_cast<RocketTypes::UvState::T>(st)); }
void ServoDriverSim::evModelRoleMismatch(ServoSim::ServoModel model, ServoSim::ServoRole role, uint8_t channel_count) {
  this->log_WARNING_HI_ModelRoleMismatch(
      static_cast<Components::ServoModel::T>(model),
      static_cast<Components::ServoRole::T>(role),
      channel_count);
}

}  // namespace Components
