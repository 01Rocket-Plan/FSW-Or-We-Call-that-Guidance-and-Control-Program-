// ======================================================================
// \title  ServoPwmHal.hpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  舵機 PWM 硬體抽象：ESP32 LEDC high-speed（50 Hz、µs）；host 為 stub
// 不 sleep；不取系統時鐘（由驅動用 F´ Time）。
// ======================================================================
#ifndef COMPONENTS_SERVO_PWM_SERVOPWMHAL_HPP
#define COMPONENTS_SERVO_PWM_SERVOPWMHAL_HPP

#include "Fw/FPrimeBasicTypes.hpp"

namespace Components {

//! 單通道：GPIO + LEDC 硬體通道號（0..7）
struct ServoPwmChannel {
  U8 gpio;    //!< GPIO；ServoPwmPins::kUnassigned = 未指派
  U8 ledcCh;  //!< LEDC channel index
};

//! 最多 4 路／實例（尾翼或 SEP）；雲台用前 2 路
class ServoPwmHal {
 public:
  static constexpr U8 kMaxChannels = 4;
  //! 解析度／頻率見 ServoPwmPins（MCU v0.1：14-bit、50 Hz、HS timer0）

  ServoPwmHal();

  //! 設定頻率、LEDC timer、通道表（可在 start 前重複呼叫）
  //! @return false 若 n==0 或 n>kMaxChannels
  bool configure(U16 freqHz, U8 ledcTimerId, const ServoPwmChannel* ch, U8 n);

  //! 套用 configure：ESP32 上 ledc_timer_config + ledc_channel_config；
  //! host stub 一律成功（略過 GPIO）。
  //! ESP：任一 gpio==kUnassigned → false。
  bool start();

  //! 寫入脈寬（µs）；idx 為 configure 順序。ESP：ledc_set_duty＋update。
  void setPulseUs(U8 idx, U16 pulseUs);

  //! 批次寫入前 n 路（n 受 channel 數限制）
  void setPulsesUs(const U16* pulseUs, U8 n);

  bool ready() const { return m_ready; }
  U8 channelCount() const { return m_n; }

 private:
  bool m_configured;
  bool m_ready;
  U16 m_freqHz;
  U8 m_timerId;
  U8 m_n;
  ServoPwmChannel m_ch[kMaxChannels];
  U16 m_lastUs[kMaxChannels];

#if defined(ESP_PLATFORM)
  U32 pulseToDuty(U16 pulseUs) const;
#endif
};

}  // namespace Components

#endif
