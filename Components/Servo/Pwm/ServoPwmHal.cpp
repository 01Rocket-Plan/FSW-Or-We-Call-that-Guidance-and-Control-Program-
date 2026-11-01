// ======================================================================
// \title  ServoPwmHal.cpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  ESP32 LEDC 實作／host stub
// ======================================================================
#include "Components/Servo/Pwm/ServoPwmHal.hpp"
#include "Components/Servo/Pwm/ServoPwmPins.hpp"

#if defined(ESP_PLATFORM)
#include "driver/ledc.h"
#endif

namespace Components {

ServoPwmHal::ServoPwmHal()
    : m_configured(false),
      m_ready(false),
      m_freqHz(ServoPwmPins::kPwmHz),
      m_timerId(0),
      m_n(0) {
  for (U8 i = 0; i < kMaxChannels; ++i) {
    m_ch[i].gpio = ServoPwmPins::kUnassigned;
    m_ch[i].ledcCh = i;
    m_lastUs[i] = 0;
  }
}

bool ServoPwmHal::configure(U16 freqHz, U8 ledcTimerId, const ServoPwmChannel* ch, U8 n) {
  if (ch == nullptr || n == 0 || n > kMaxChannels) {
    return false;
  }
  if (freqHz == 0) {
    freqHz = ServoPwmPins::kPwmHz;
  }
  m_freqHz = freqHz;
  m_timerId = ledcTimerId;
  m_n = n;
  for (U8 i = 0; i < n; ++i) {
    m_ch[i] = ch[i];
  }
  m_configured = true;
  m_ready = false;
  return true;
}

#if defined(ESP_PLATFORM)

U32 ServoPwmHal::pulseToDuty(U16 pulseUs) const {
  // duty = pulse_us / period_us * (2^res)
  // period_us = 1e6 / freqHz
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

bool ServoPwmHal::start() {
  m_ready = false;
  if (!m_configured || m_n == 0) {
    return false;
  }
  for (U8 i = 0; i < m_n; ++i) {
    if (m_ch[i].gpio == ServoPwmPins::kUnassigned || m_ch[i].ledcCh > 7) {
      return false;
    }
  }

  // MCU 組 v0.1：僅 high-speed + timer 0（見 ServoPwmPins）
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
    ccfg.channel = static_cast<ledc_channel_t>(m_ch[i].ledcCh);
    ccfg.intr_type = LEDC_INTR_DISABLE;
    ccfg.timer_sel = static_cast<ledc_timer_t>(m_timerId);
    ccfg.duty = 0;
    ccfg.hpoint = 0;
    if (ledc_channel_config(&ccfg) != ESP_OK) {
      return false;
    }
  }
  m_ready = true;
  return true;
}

void ServoPwmHal::setPulseUs(U8 idx, U16 pulseUs) {
  if (!m_ready || idx >= m_n) {
    return;
  }
  m_lastUs[idx] = pulseUs;
  const U32 duty = pulseToDuty(pulseUs);
  const ledc_channel_t ch = static_cast<ledc_channel_t>(m_ch[idx].ledcCh);
  const ledc_mode_t speed = static_cast<ledc_mode_t>(ServoPwmPins::kLedcSpeedMode);
  (void)ledc_set_duty(speed, ch, duty);
  (void)ledc_update_duty(speed, ch);
}

#else  // host stub

bool ServoPwmHal::start() {
  // Host／CI：不碰硬體；略過 GPIO 未指派，只要 configure 過即就緒
  if (!m_configured || m_n == 0) {
    m_ready = false;
    return false;
  }
  m_ready = true;
  return true;
}

void ServoPwmHal::setPulseUs(U8 idx, U16 pulseUs) {
  if (!m_ready || idx >= m_n) {
    return;
  }
  m_lastUs[idx] = pulseUs;
  // no-op hardware
}

#endif

void ServoPwmHal::setPulsesUs(const U16* pulseUs, U8 n) {
  if (pulseUs == nullptr) {
    return;
  }
  const U8 lim = (n < m_n) ? n : m_n;
  for (U8 i = 0; i < lim; ++i) {
    setPulseUs(i, pulseUs[i]);
  }
}

}  // namespace Components
