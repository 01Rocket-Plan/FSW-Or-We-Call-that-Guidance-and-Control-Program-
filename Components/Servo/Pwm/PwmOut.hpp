// ======================================================================
// \title  PwmOut.hpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  抽象 PWM 輸出介面；驅動只呼叫本類，不碰 LEDC／TIMx 等後端 API
//
// 契約：init(ch, pin, freq_hz)／setPulse_us(ch, us)／disable(ch)
// Backend A（現行）：ESP32 LEDC high-speed；host stub。
// Backend B（設計見 v0.5 文件，程式尚未實作）：目標 MCU 硬體 timer PWM，
//   否則 timer-ISR 軟體 PWM。
// 不 sleep；不取系統時鐘（由驅動用 F´ Time）。
// ======================================================================
#ifndef COMPONENTS_SERVO_PWM_PWMOUT_HPP
#define COMPONENTS_SERVO_PWM_PWMOUT_HPP

#include "Fw/FPrimeBasicTypes.hpp"

namespace Components {

//! 單通道板級設定（腳位＋後端通道號；ESP＝LEDC ch）
struct PwmOutChannel {
  U8 gpio;   //!< GPIO；ServoPwmPins::kUnassigned = 未指派
  U8 hwCh;   //!< 後端通道（ESP LEDC 0..7）
};

//! 相容舊名
using ServoPwmChannel = PwmOutChannel;

//! 抽象 PwmOut（最多 4 路／實例；雲台用前 2 路）
class PwmOut {
 public:
  static constexpr U8 kMaxChannels = 4;

  PwmOut();

  //! 載入板級通道表（腳位／hwCh／timer）；尚不碰硬體
  bool configure(U16 freqHz, U8 timerId, const PwmOutChannel* ch, U8 n);

  //! 抽象 init(pin, freq_hz)：以通道索引綁定腳位並套用頻率（可重複呼叫）
  //! 全部所需通道 init 完後須再呼叫 init()（無參／僅頻率）以啟動後端，
  //! 或直接用下方批次 init(freqHz)（板級表已 configure 時）。
  bool init(U8 ch, U8 pin, U16 freq_hz);

  //! 批次啟動後端（板級表已 configure）；≡ 舊 start()
  bool init(U16 freq_hz);
  bool init();  //!< 使用已 configure 的頻率

  //! 寫入脈寬（µs）
  void setPulse_us(U8 ch, U16 us);

  //! 關閉該通道輸出（duty=0／停波）；不釋放板級設定
  void disable(U8 ch);

  //! 批次寫入
  void setPulses_us(const U16* pulseUs, U8 n);

  bool ready() const { return m_ready; }
  U8 channelCount() const { return m_n; }

  // ---- 過渡別名（舊呼叫點）----
  bool start() { return init(); }
  void setPulseUs(U8 ch, U16 us) { setPulse_us(ch, us); }
  void setPulsesUs(const U16* us, U8 n) { setPulses_us(us, n); }

 private:
  bool startBackend();

  bool m_configured;
  bool m_ready;
  U16 m_freqHz;
  U8 m_timerId;
  U8 m_n;
  PwmOutChannel m_ch[kMaxChannels];
  U16 m_lastUs[kMaxChannels];
  bool m_enabled[kMaxChannels];

#if defined(ESP_PLATFORM)
  U32 pulseToDuty(U16 pulseUs) const;
#endif
};

//! 相容舊類名
using ServoPwmHal = PwmOut;

}  // namespace Components

#endif
