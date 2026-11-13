// ======================================================================
// \title  ServoPwmPins.hpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  LEDC GPIO／通道／timer 設定常數（勿寫死在驅動內）
//
// MCU 組 v0.1 草案。全箭腳位表由 MCU 組（EdwardWang+Claude）在 ESP32 平台 PR 統一整理
// （Edward 2026-11-10：腳位自訂、不衝突即可）。
// ======================================================================
#ifndef COMPONENTS_SERVO_PWM_SERVOPWMPINS_HPP
#define COMPONENTS_SERVO_PWM_SERVOPWMPINS_HPP

#include "Fw/FPrimeBasicTypes.hpp"

namespace Components {
namespace ServoPwmPins {

//! 未指派腳位（勿用 0：ESP32 GPIO0 為合法腳）
constexpr U8 kUnassigned = 0xFF;

// ----------------------------------------------------------------------
// 全域 LEDC 政策（MCU 組 v0.1 草案，pending Anatoly review）
// 全部 high-speed；共用 timer 0；禁止 low-speed 通道／timer 1–3
// ----------------------------------------------------------------------
constexpr U8 kLedcSpeedMode = 0;     //!< 對應 LEDC_HIGH_SPEED_MODE（禁止 LOW_SPEED）
constexpr U8 kLedcTimerShared = 0;   //!< 唯一允許的 timer；勿用 1–3
constexpr U16 kPwmHz = 50;
constexpr U8 kResBits = 14;

//! DS 尾翼 SG90（ServoDriver base 0x3000）— TAIL_1..4
//! MCU 組 v0.1 草案，pending Anatoly review：GPIO 25,26,27,14；HS ch 0–3
constexpr U8 TAIL_GPIO[4] = {25, 26, 27, 14};
constexpr U8 TAIL_LEDC_CH[4] = {0, 1, 2, 3};
constexpr U8 TAIL_LEDC_TIMER = kLedcTimerShared;

//! DS 級間分離 MG996R（SepServoDriver base 0x3100）— SEP_1..3（3 顆環排 120°，SSS §7）
//! MCU 組 v0.1 草案：GPIO 32,33,13；HS ch 4–6（原 SEP_4 的 GPIO4／ch7 釋出）
constexpr U8 SEP_GPIO[3] = {32, 33, 13};
constexpr U8 SEP_LEDC_CH[3] = {4, 5, 6};
constexpr U8 SEP_LEDC_TIMER = kLedcTimerShared;

//! US 雲台 SG90（ServoDriver 實例 base 0x3200）— PITCH／ROLL
//! MCU 組 v0.1 草案，pending Anatoly review：PITCH GPIO25 ch0；ROLL GPIO26 ch1
constexpr U8 GIMBAL_GPIO[2] = {25, 26};
constexpr U8 GIMBAL_LEDC_CH[2] = {0, 1};
constexpr U8 GIMBAL_LEDC_TIMER = kLedcTimerShared;

}  // namespace ServoPwmPins
}  // namespace Components

#endif
