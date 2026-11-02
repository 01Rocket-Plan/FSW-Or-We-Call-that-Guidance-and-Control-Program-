// ======================================================================
// \title  PwmOut.cpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  Backend A：ESP32 LEDC／host stub（驅動不直接 include ledc.h）
// ======================================================================
#include "Components/Servo/Pwm/PwmOut.hpp"
#include "Components/Servo/Pwm/ServoPwmPins.hpp"

#if defined(ESP_PLATFORM)
#include "driver/ledc.h"
#endif

namespace Components {

PwmOut::PwmOut()
    : m_configured(false),
      m_ready(false),
      m_freqHz(ServoPwmPins::kPwmHz),
      m_timerId(ServoPwmPins::kLedcTimerShared),
      m_n(0) {
  for (U8 i = 0; i < kMaxChannels; ++i) {
    m_ch[i].gpio = ServoPwmPins::kUnassigned;
    m_ch[i].hwCh = i;
    m_lastUs[i] = 0;
    m_enabled[i] = false;
  }
}

bool PwmOut::configure(U16 freqHz, U8 timerId, const PwmOutChannel* ch, U8 n) {
  if (ch == nullptr || n == 0 || n > kMaxChannels) {
    return false;
  }
  if (freqHz == 0) {
    freqHz = ServoPwmPins::kPwmHz;
  }
  m_freqHz = freqHz;
  m_timerId = timerId;
  m_n = n;
  for (U8 i = 0; i < n; ++i) {
    m_ch[i] = ch[i];
    m_enabled[i] = false;
  }
  m_configured = true;
  m_ready = false;
  return true;
}

bool PwmOut::init(U8 ch, U8 pin, U16 freq_hz) {
  if (ch >= kMaxChannels) {
    return false;
  }
  if (freq_hz == 0) {
    freq_hz = ServoPwmPins::kPwmHz;
  }
  m_freqHz = freq_hz;
  m_ch[ch].gpio = pin;
  if (m_ch[ch].hwCh > 7) {
    m_ch[ch].hwCh = ch;
  }
  if (m_n <= ch) {
    m_n = static_cast<U8>(ch + 1);
  }
  m_configured = true;
  m_ready = false;
  // Defer hardware bring-up to batch init() so shared timer is programmed once
  return true;
}

bool PwmOut::init(U16 freq_hz) {
  if (freq_hz != 0) {
    m_freqHz = freq_hz;
  }
  return startBackend();
}

bool PwmOut::init() { return startBackend(); }

#if defined(ESP_PLATFORM)

U32 PwmOut::pulseToDuty(U16 pulseUs) const {
  const U32 maxDuty = (1u << ServoPwmPins::kResBits);
  const U32 periodUs = 1000000u / static_cast<U32>(m_freqHz == 0 ? ServoPwmPins::kPwmHz : m_freqHz);
  if (periodUs == 0) {
    return 0;
  }
  U32 duty = (static_cast<U32>(pulseUs) * maxDuty) / periodUs;
  if (duty >= maxDuty) {
    duty = maxDuty - 1u;
  }
  return duty;
}

bool PwmOut::startBackend() {
  m_ready = false;
  if (!m_configured || m_n == 0) {
    return false;
  }
  for (U8 i = 0; i < m_n; ++i) {
    if (m_ch[i].gpio == ServoPwmPins::kUnassigned || m_ch[i].hwCh > 7) {
      return false;
    }
  }

  // Backend A — ESP32 LEDC HS + shared timer0（ServoPwmPins）
  const ledc_mode_t speed = static_cast<ledc_mode_t>(ServoPwmPins::kLedcSpeedMode);
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode = speed;
  tcfg.duty_resolution = static_cast<ledc_timer_bit_t>(ServoPwmPins::kResBits);
  tcfg.timer_num = static_cast<ledc_timer_t>(m_timerId);
  tcfg.freq_hz = m_freqHz;
  tcfg.clk_cfg = LEDC_AUTO_CLK;
  if (ledc_timer_config(&tcfg) != ESP_OK) {
    return false;
  }

  for (U8 i = 0; i < m_n; ++i) {
    ledc_channel_config_t ccfg = {};
    ccfg.gpio_num = static_cast<int>(m_ch[i].gpio);
    ccfg.speed_mode = speed;
    ccfg.channel = static_cast<ledc_channel_t>(m_ch[i].hwCh);
    ccfg.intr_type = LEDC_INTR_DISABLE;
    ccfg.timer_sel = static_cast<ledc_timer_t>(m_timerId);
    ccfg.duty = 0;
    ccfg.hpoint = 0;
    if (ledc_channel_config(&ccfg) != ESP_OK) {
      return false;
    }
    m_enabled[i] = true;
  }
  m_ready = true;
  return true;
}

void PwmOut::setPulse_us(U8 ch, U16 us) {
  if (!m_ready || ch >= m_n || !m_enabled[ch]) {
    return;
  }
  m_lastUs[ch] = us;
  const U32 duty = pulseToDuty(us);
  const ledc_channel_t hw = static_cast<ledc_channel_t>(m_ch[ch].hwCh);
  const ledc_mode_t speed = static_cast<ledc_mode_t>(ServoPwmPins::kLedcSpeedMode);
  (void)ledc_set_duty(speed, hw, duty);
  (void)ledc_update_duty(speed, hw);
}

void PwmOut::disable(U8 ch) {
  if (ch >= m_n) {
    return;
  }
  m_enabled[ch] = false;
  m_lastUs[ch] = 0;
  if (!m_ready) {
    return;
  }
  const ledc_channel_t hw = static_cast<ledc_channel_t>(m_ch[ch].hwCh);
  const ledc_mode_t speed = static_cast<ledc_mode_t>(ServoPwmPins::kLedcSpeedMode);
  (void)ledc_set_duty(speed, hw, 0);
  (void)ledc_update_duty(speed, hw);
}

#else  // host stub (Backend A stand-in for CI)

bool PwmOut::startBackend() {
  if (!m_configured || m_n == 0) {
    m_ready = false;
    return false;
  }
  for (U8 i = 0; i < m_n; ++i) {
    m_enabled[i] = true;
  }
  m_ready = true;
  return true;
}

void PwmOut::setPulse_us(U8 ch, U16 us) {
  if (!m_ready || ch >= m_n || !m_enabled[ch]) {
    return;
  }
  m_lastUs[ch] = us;
}

void PwmOut::disable(U8 ch) {
  if (ch >= m_n) {
    return;
  }
  m_enabled[ch] = false;
  m_lastUs[ch] = 0;
}

#endif

void PwmOut::setPulses_us(const U16* pulseUs, U8 n) {
  if (pulseUs == nullptr) {
    return;
  }
  const U8 lim = (n < m_n) ? n : m_n;
  for (U8 i = 0; i < lim; ++i) {
    setPulse_us(i, pulseUs[i]);
  }
}

}  // namespace Components
