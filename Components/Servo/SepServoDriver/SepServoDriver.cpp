#include "Components/Servo/SepServoDriver/SepServoDriver.hpp"
#include "Components/Servo/Pwm/ServoPwmPins.hpp"
#include <cstring>

namespace Components {

namespace {
enum DropReason : U8 { DROP_NOT_READY = 1, DROP_SEQ = 2, DROP_INVALID = 3 };
enum FaultReason : U8 { FAULT_SELFTEST = 1 };
constexpr U8 kSepBits = 0x0F;
}  // namespace

SepServoDriver::SepServoDriver(const char* compName)
    : SepServoDriverComponentBase(compName),
      m_state(RocketTypes::DrvState::OFF),
      m_uv(RocketTypes::UvState::NOMINAL),
      m_lastSeq(0),
      m_haveSeq(false),
      m_activeMask(0),
      m_lockedMask(kSepBits),
      m_tlmDiv(0),
      m_stateEnter_s(0.0),
      m_pwmConfigured(false),
      m_ledcTimerId(ServoPwmPins::SEP_LEDC_TIMER),
      m_gpioN(0),
      m_lock_us(1000),
      m_fire_us(2000),
      m_fireHold_s(0.5f),
      m_min_us(1000),
      m_max_us(2000),
      m_selftestDeadline_s(1.0f) {
  for (U8 i = 0; i < 4; ++i) {
    m_out_us[i] = 1000;
    m_fireStart_s[i] = 0.0;
    m_chCfg[i].gpio = ServoPwmPins::SEP_GPIO[i];
    m_chCfg[i].ledcCh = ServoPwmPins::SEP_LEDC_CH[i];
  }
  m_gpioN = 4;
}

void SepServoDriver::configure(const U8* gpioPins, U8 nPins) {
  m_gpioN = (nPins > ServoPwmHal::kMaxChannels) ? ServoPwmHal::kMaxChannels : nPins;
  for (U8 i = 0; i < m_gpioN; ++i) {
    m_chCfg[i].gpio = gpioPins[i];
    m_chCfg[i].ledcCh = ServoPwmPins::SEP_LEDC_CH[i];
  }
  m_pwmConfigured = false;
}

void SepServoDriver::configure(const ServoPwmChannel* ch, U8 n, U8 ledcTimerId) {
  if (ch == nullptr || n == 0) {
    return;
  }
  m_gpioN = (n > ServoPwmHal::kMaxChannels) ? ServoPwmHal::kMaxChannels : n;
  for (U8 i = 0; i < m_gpioN; ++i) {
    m_chCfg[i] = ch[i];
  }
  m_ledcTimerId = ledcTimerId;
  m_pwmConfigured = false;
}

void SepServoDriver::loadParams() {
  Fw::ParamValid v;
  m_lock_us = this->paramGet_lock_us(v);
  m_fire_us = this->paramGet_fire_us(v);
  m_fireHold_s = this->paramGet_fire_hold_s(v);
  m_min_us = this->paramGet_pulse_min_us(v);
  m_max_us = this->paramGet_pulse_max_us(v);
  m_selftestDeadline_s = this->paramGet_selftest_deadline_s(v);
}

void SepServoDriver::parameterUpdated(FwPrmIdType) { loadParams(); }
void SepServoDriver::parametersLoaded() { loadParams(); }

F64 SepServoDriver::now_s() const {
  const Fw::Time t = this->getTime();
  return static_cast<F64>(t.getSeconds()) + static_cast<F64>(t.getUSeconds()) * 1e-6;
}

void SepServoDriver::enterState(RocketTypes::DrvState::T st, U8 faultReason) {
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
    // SepFaultHold ONLY on FAULT entry: unreleased stay at lock_us
    const U8 fired = readFiredMask();
    m_lockedMask = static_cast<U8>((~fired) & kSepBits);
    for (U8 i = 0; i < 4; ++i) {
      if ((m_lockedMask & (1u << i)) != 0) {
        m_out_us[i] = clampUs(m_lock_us);
        m_activeMask = static_cast<U8>(m_activeMask & ~(1u << i));
      }
    }
    this->log_WARNING_HI_SepFaultHold(m_lockedMask);
    applyPwm();
  }
}

void SepServoDriver::ensurePwmConfigured() {
  U8 n = (m_gpioN > 0) ? m_gpioN : static_cast<U8>(4);
  if (n > ServoPwmHal::kMaxChannels) {
    n = ServoPwmHal::kMaxChannels;
  }
  (void)m_pwm.configure(ServoPwmPins::kPwmHz, m_ledcTimerId, m_chCfg, n);
}

U16 SepServoDriver::clampUs(U16 raw) const {
  if (raw < m_min_us) {
    return m_min_us;
  }
  if (raw > m_max_us) {
    return m_max_us;
  }
  return raw;
}

U8 SepServoDriver::readFiredMask() {
  U8 mask = 0;
  if (this->isConnected_sepReleasedGet_OutputPort(0)) {
    this->sepReleasedGet_out(0, mask);
  }
  return static_cast<U8>(mask & kSepBits);
}

bool SepServoDriver::tryFireChannel(U8 ch) {
  const U8 bit = static_cast<U8>(1u << ch);
  const U8 fired = readFiredMask();
  if ((fired & bit) != 0) {
    this->log_WARNING_LO_SepAlreadyFired(ch);
    return false;
  }
  if (!this->isConnected_sepReleasedSet_OutputPort(0)) {
    // No NvFlags connected: refuse fire (cannot guarantee once-only)
    this->log_WARNING_HI_SepNvWriteFail(ch);
    m_out_us[ch] = clampUs(m_lock_us);
    return false;
  }
  const Fw::Success st = this->sepReleasedSet_out(0, SepNvPlaceholder::NvSepSet(bit));
  if (st != Fw::Success::SUCCESS) {
    // Do NOT enter FAULT; keep lock_us; later cmd may retry
    this->log_WARNING_HI_SepNvWriteFail(ch);
    m_out_us[ch] = clampUs(m_lock_us);
    return false;
  }
  m_out_us[ch] = clampUs(m_fire_us);
  m_activeMask = static_cast<U8>(m_activeMask | bit);
  m_lockedMask = static_cast<U8>(m_lockedMask & ~bit);
  m_fireStart_s[ch] = now_s();
  this->log_ACTIVITY_HI_SepFire(ch);
  return true;
}

void SepServoDriver::emitPwmOut() {
  if (!this->isConnected_pwmOut_OutputPort(0)) {
    return;
  }
  const U8 mask = static_cast<U8>(m_activeMask | readFiredMask());
  this->pwmOut_out(0, RocketTypes::SepCmd(now_s(), m_lastSeq, mask, true));
}

void SepServoDriver::applyPwm() {
  m_pwm.setPulsesUs(m_out_us, 4);
  emitPwmOut();
}

void SepServoDriver::writeTlm() {
  this->tlmWrite_drvState(RocketTypes::DrvState(m_state));
  this->tlmWrite_lastSeq(m_lastSeq);
  this->tlmWrite_firedMask(readFiredMask());
  this->tlmWrite_lockedMask(m_lockedMask);
  this->tlmWrite_uvState(m_uv);
}

void SepServoDriver::uvIn_handler(FwIndexType, const RocketTypes::UvState& st) {
  m_uv = st;
  this->tlmWrite_uvState(st);
  this->log_ACTIVITY_HI_UvSeen(st);
}

void SepServoDriver::cmdIn_handler(FwIndexType, const RocketTypes::SepCmd& c) {
  if (m_state != RocketTypes::DrvState::READY) {
    this->log_WARNING_LO_CmdDropped(c.get_seq(), DROP_NOT_READY);
    return;
  }
  if (!c.get_valid()) {
    this->log_WARNING_LO_CmdDropped(c.get_seq(), DROP_INVALID);
    return;
  }
  if (m_haveSeq && c.get_seq() <= m_lastSeq) {
    this->log_WARNING_LO_CmdDropped(c.get_seq(), DROP_SEQ);
    return;
  }
  m_haveSeq = true;
  m_lastSeq = c.get_seq();

  const U8 req = static_cast<U8>(c.get_channelMask() & kSepBits);
  for (U8 i = 0; i < 4; ++i) {
    if ((req & (1u << i)) != 0) {
      (void)tryFireChannel(i);
    }
  }
  applyPwm();
}

void SepServoDriver::schedIn_handler(FwIndexType, U32) {
  const F64 t = now_s();

  switch (m_state) {
    case RocketTypes::DrvState::OFF:
      enterState(RocketTypes::DrvState::INIT);
      break;
    case RocketTypes::DrvState::INIT:
      ensurePwmConfigured();
      m_pwmConfigured = m_pwm.start();
      for (U8 i = 0; i < 4; ++i) {
        m_out_us[i] = clampUs(m_lock_us);
      }
      enterState(RocketTypes::DrvState::SELFTEST);
      break;
    case RocketTypes::DrvState::SELFTEST:
      if (m_pwmConfigured && m_pwm.ready() && m_min_us < m_max_us) {
        // Seed lockedMask from NV (already-fired stay released state)
        const U8 fired = readFiredMask();
        m_lockedMask = static_cast<U8>((~fired) & kSepBits);
        applyPwm();
        enterState(RocketTypes::DrvState::READY);
      } else if ((t - m_stateEnter_s) >= static_cast<F64>(m_selftestDeadline_s)) {
        enterState(RocketTypes::DrvState::FAULT, FAULT_SELFTEST);
      }
      break;
    case RocketTypes::DrvState::READY:
      // After fire_hold_s, leave hardware at last fire pulse (no re-fire); do not return to lock
      for (U8 i = 0; i < 4; ++i) {
        const U8 bit = static_cast<U8>(1u << i);
        if ((m_activeMask & bit) != 0) {
          if ((t - m_fireStart_s[i]) >= static_cast<F64>(m_fireHold_s)) {
            m_activeMask = static_cast<U8>(m_activeMask & ~bit);
            // keep m_out_us[i] at fire_us (released channel keeps state)
          }
        }
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
