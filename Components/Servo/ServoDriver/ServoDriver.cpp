#include "Components/Servo/ServoDriver/ServoDriver.hpp"
#include "Components/Servo/Pwm/ServoPwmPins.hpp"
#include <cstring>

namespace Components {

namespace {
enum DropReason : U8 { DROP_NOT_READY = 1, DROP_SEQ = 2 };
enum FaultReason : U8 { FAULT_SELFTEST = 1, FAULT_PWM = 2, FAULT_MODEL_ROLE = 3 };
enum LockSource : U8 { LOCK_CMD = 1, LOCK_TIMEOUT = 2 };
}  // namespace

ServoDriver::ServoDriver(const char* compName)
    : ServoDriverComponentBase(compName),
      m_state(RocketTypes::DrvState::OFF),
      m_uv(RocketTypes::UvState::NOMINAL),
      m_lastSeq(0),
      m_haveSeq(false),
      m_lastCmd_s(0.0),
      m_haveCmd(false),
      m_lockNeutral(true),
      m_tick(0),
      m_pwmPeriodTicks(20),
      m_tlmDiv(0),
      m_clampCount(0),
      m_dropCount(0),
      m_stateEnter_s(0.0),
      m_pwmConfigured(false),
      m_ledcTimerId(ServoPwmPins::TAIL_LEDC_TIMER),
      m_gpioN(0),
      m_neutral_us(1500),
      m_min_us(1000),
      m_max_us(2000),
      m_cmdTimeout_s(0.05f),
      m_pwmHz(50),
      m_channelCount(4),
      m_selftestDeadline_s(1.0f),
      m_model(Components::ServoModel::SG90),
      m_role(Components::ServoRole::TAIL) {
  for (U16& p : m_out_us) {
    p = 1500;
  }
  for (U8 i = 0; i < PwmOut::kMaxChannels; ++i) {
    m_chCfg[i].gpio = ServoPwmPins::TAIL_GPIO[i];
    m_chCfg[i].hwCh = ServoPwmPins::TAIL_LEDC_CH[i];
  }
  m_gpioN = 4;
}

void ServoDriver::configure(const U8* gpioPins, U8 nPins) {
  m_gpioN = (nPins > PwmOut::kMaxChannels) ? PwmOut::kMaxChannels : nPins;
  for (U8 i = 0; i < m_gpioN; ++i) {
    m_chCfg[i].gpio = gpioPins[i];
    m_chCfg[i].hwCh = ServoPwmPins::TAIL_LEDC_CH[i];
  }
  m_pwmConfigured = false;
}

void ServoDriver::configure(const PwmOutChannel* ch, U8 n, U8 ledcTimerId) {
  if (ch == nullptr || n == 0) {
    return;
  }
  m_gpioN = (n > PwmOut::kMaxChannels) ? PwmOut::kMaxChannels : n;
  for (U8 i = 0; i < m_gpioN; ++i) {
    m_chCfg[i] = ch[i];
  }
  m_ledcTimerId = ledcTimerId;
  m_pwmConfigured = false;
}

void ServoDriver::loadParams() {
  Fw::ParamValid v;
  m_model = this->paramGet_model(v);
  m_role = this->paramGet_role(v);
  m_neutral_us = this->paramGet_neutral_us(v);
  m_min_us = this->paramGet_pulse_min_us(v);
  m_max_us = this->paramGet_pulse_max_us(v);
  m_cmdTimeout_s = this->paramGet_cmd_timeout_s(v);
  m_pwmHz = this->paramGet_pwm_hz(v);
  m_channelCount = this->paramGet_channel_count(v);
  m_selftestDeadline_s = this->paramGet_selftest_deadline_s(v);
  if (m_channelCount == 0 || m_channelCount > 4) {
    m_channelCount = 4;
  }
  if (m_pwmHz == 0) {
    m_pwmHz = 50;
  }
  // sched assumed 1 kHz (rg1kHz)
  m_pwmPeriodTicks = static_cast<U32>(1000u / m_pwmHz);
  if (m_pwmPeriodTicks == 0) {
    m_pwmPeriodTicks = 1;
  }
}

void ServoDriver::parameterUpdated(FwPrmIdType) { loadParams(); }
void ServoDriver::parametersLoaded() { loadParams(); }

F64 ServoDriver::now_s() const {
  const Fw::Time t = this->getTime();
  return static_cast<F64>(t.getSeconds()) + static_cast<F64>(t.getUSeconds()) * 1e-6;
}

void ServoDriver::enterState(RocketTypes::DrvState::T st, U8 faultReason) {
  if (m_state == st) {
    return;
  }
  m_state = st;
  m_stateEnter_s = now_s();
  this->tlmWrite_drvState(RocketTypes::DrvState(st));
  if (st == RocketTypes::DrvState::READY) {
    this->log_ACTIVITY_HI_EnterReady();
  } else if (st == RocketTypes::DrvState::FAULT) {
    this->log_WARNING_HI_EnterFault(faultReason);
  }
}

void ServoDriver::ensurePwmConfigured() {
  U8 n = m_channelCount;
  if (n == 0 || n > PwmOut::kMaxChannels) {
    n = m_gpioN;
  }
  if (n == 0) {
    n = 4;
  }
  if (m_gpioN > 0 && n > m_gpioN) {
    n = m_gpioN;
  }
  (void)m_pwm.configure(m_pwmHz, m_ledcTimerId, m_chCfg, n);
}

bool ServoDriver::pwmSelftestOk() const {
  return m_pwmConfigured && m_pwm.ready() && m_channelCount >= 1 && m_channelCount <= 4 &&
         m_pwmHz >= 20 && m_pwmHz <= 400 && m_min_us < m_max_us;
}

bool ServoDriver::modelRoleOk() const {
  // Formal identity: (SG90,TAIL,4) or (SG90,GIMBAL,2) only
  if (m_model != Components::ServoModel::SG90) {
    return false;
  }
  if (m_role == Components::ServoRole::TAIL && m_channelCount == 4) {
    return true;
  }
  if (m_role == Components::ServoRole::GIMBAL && m_channelCount == 2) {
    return true;
  }
  return false;
}

U16 ServoDriver::clampPulse(U8 ch, U16 raw) {
  U16 used = raw;
  if (raw < m_min_us) {
    used = m_min_us;
  } else if (raw > m_max_us) {
    used = m_max_us;
  }
  if (used != raw) {
    ++m_clampCount;
    this->log_WARNING_LO_PulseClamped(ch, raw, used);
  }
  return used;
}

// 伺服馬達只吃 PWM 訊號，只負責接收 MCU 指令並作動；與整機 I2C 通訊協議分屬不同層級，不衝突。
void ServoDriver::emitPwmOut() {
  if (!this->isConnected_pwmOut_OutputPort(0)) {
    return;
  }
  RocketTypes::ServoCmd::Type_of_pulse_us pulses = {m_out_us[0], m_out_us[1], m_out_us[2], m_out_us[3]};
  this->pwmOut_out(0, RocketTypes::ServoCmd(now_s(), m_lastSeq, pulses, m_lockNeutral));
}

void ServoDriver::applyOutputs(bool /*emitTlm*/) {
  m_pwm.setPulses_us(m_out_us, m_channelCount);
  emitPwmOut();
}

void ServoDriver::writeTlm() {
  this->tlmWrite_drvState(RocketTypes::DrvState(m_state));
  this->tlmWrite_lastSeq(m_lastSeq);
  this->tlmWrite_lastPulse0_us(m_out_us[0]);
  this->tlmWrite_lastPulse1_us(m_out_us[1]);
  this->tlmWrite_lastPulse2_us(m_out_us[2]);
  this->tlmWrite_lastPulse3_us(m_out_us[3]);
  F32 age = 0.0f;
  if (m_haveCmd) {
    age = static_cast<F32>(now_s() - m_lastCmd_s);
  }
  this->tlmWrite_cmdAge_s(age);
  this->tlmWrite_uvState(m_uv);
  this->tlmWrite_clampCount(m_clampCount);
  this->tlmWrite_dropCount(m_dropCount);
  this->tlmWrite_model(m_model);
  this->tlmWrite_role(m_role);
}

void ServoDriver::uvIn_handler(FwIndexType, const RocketTypes::UvState& st) {
  m_uv = st;
  this->tlmWrite_uvState(st);
  this->log_ACTIVITY_HI_UvSeen(st);
}

void ServoDriver::cmdIn_handler(FwIndexType, const RocketTypes::ServoCmd& c) {
  if (m_state != RocketTypes::DrvState::READY) {
    ++m_dropCount;
    this->log_WARNING_LO_CmdDropped(c.get_seq(), DROP_NOT_READY);
    return;
  }
  if (m_haveSeq && c.get_seq() <= m_lastSeq) {
    ++m_dropCount;
    this->log_WARNING_LO_CmdDropped(c.get_seq(), DROP_SEQ);
    return;
  }
  m_haveSeq = true;
  m_lastSeq = c.get_seq();
  m_haveCmd = true;
  m_lastCmd_s = now_s();  // prefer F´ time; cmd.t_s available but age uses service time
  m_lockNeutral = c.get_lockNeutral();
  if (m_lockNeutral) {
    this->log_ACTIVITY_HI_LockNeutral(LOCK_CMD);
    for (U8 i = 0; i < m_channelCount; ++i) {
      m_out_us[i] = m_neutral_us;
    }
  } else {
    const auto& pulses = c.get_pulse_us();
    for (U8 i = 0; i < m_channelCount; ++i) {
      m_out_us[i] = clampPulse(i, pulses[i]);
    }
  }
  for (U8 i = m_channelCount; i < 4; ++i) {
    m_out_us[i] = m_neutral_us;
  }
  applyOutputs(false);
}

void ServoDriver::schedIn_handler(FwIndexType, U32) {
  const F64 t = now_s();
  ++m_tick;

  switch (m_state) {
    case RocketTypes::DrvState::OFF:
      enterState(RocketTypes::DrvState::INIT);
      break;
    case RocketTypes::DrvState::INIT:
      ensurePwmConfigured();
      m_pwmConfigured = m_pwm.init();
      enterState(RocketTypes::DrvState::SELFTEST);
      break;
    case RocketTypes::DrvState::SELFTEST:
      if (!modelRoleOk()) {
        this->log_WARNING_HI_ModelRoleMismatch(m_model, m_role, m_channelCount);
        enterState(RocketTypes::DrvState::FAULT, FAULT_MODEL_ROLE);
      } else if (pwmSelftestOk()) {
        for (U8 i = 0; i < 4; ++i) {
          m_out_us[i] = m_neutral_us;
        }
        m_lockNeutral = true;
        applyOutputs(false);
        this->tlmWrite_model(m_model);
        this->tlmWrite_role(m_role);
        enterState(RocketTypes::DrvState::READY);
      } else if ((t - m_stateEnter_s) >= static_cast<F64>(m_selftestDeadline_s)) {
        enterState(RocketTypes::DrvState::FAULT, FAULT_SELFTEST);
      }
      break;
    case RocketTypes::DrvState::READY:
      if (m_haveCmd) {
        const F32 age = static_cast<F32>(t - m_lastCmd_s);
        if (age > m_cmdTimeout_s) {
          m_lockNeutral = true;
          for (U8 i = 0; i < m_channelCount; ++i) {
            m_out_us[i] = m_neutral_us;
          }
          this->log_WARNING_HI_CmdTimeout(age);
          this->log_ACTIVITY_HI_LockNeutral(LOCK_TIMEOUT);
          m_haveCmd = false;  // one-shot until next cmd
          applyOutputs(false);
        }
      }
      if ((m_tick % m_pwmPeriodTicks) == 0) {
        applyOutputs(false);  // 50 Hz ZOH refresh
      }
      break;
    case RocketTypes::DrvState::FAULT:
    default:
      break;
  }

  if (++m_tlmDiv >= 1000) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

}  // namespace Components
