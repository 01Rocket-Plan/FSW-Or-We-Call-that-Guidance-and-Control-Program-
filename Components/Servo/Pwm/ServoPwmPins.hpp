// ======================================================================
// \title  ServoPwmPins.hpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  LEDC GPIO／通道預設表
//
// 倉庫文件（Deployments／README／conventions）未載明實體腳位。
// TODO(MCU／配線): 上板前填入實際 GPIO；未填則 ESP32 init 失敗（host stub 不檢查）。
// ======================================================================
#ifndef COMPONENTS_SERVO_PWM_SERVOPWMPINS_HPP
#define COMPONENTS_SERVO_PWM_SERVOPWMPINS_HPP

#include "Fw/FPrimeBasicTypes.hpp"

namespace Components {
namespace ServoPwmPins {

//! 未指派腳位（勿用 0：ESP32 GPIO0 為合法腳）
constexpr U8 kUnassigned = 0xFF;

//! DS 尾翼 SG90（ServoDriver base 0x3000）— 4 通道
//! TODO(MCU／配線): TAIL_1..4 GPIO
constexpr U8 TAIL_GPIO[4] = {kUnassigned, kUnassigned, kUnassigned, kUnassigned};
constexpr U8 TAIL_LEDC_CH[4] = {0, 1, 2, 3};
constexpr U8 TAIL_LEDC_TIMER = 0;

//! DS 級間分離 MG996R（SepServoDriver base 0x3100）— SEP_1..4
//! TODO(MCU／配線): SEP_1..4 GPIO（與尾翼分腳）
constexpr U8 SEP_GPIO[4] = {kUnassigned, kUnassigned, kUnassigned, kUnassigned};
constexpr U8 SEP_LEDC_CH[4] = {4, 5, 6, 7};
constexpr U8 SEP_LEDC_TIMER = 1;

//! US 雲台 SG90（ServoDriver 實例 base 0x3200）— pitch／roll
//! TODO(MCU／配線): GIMBAL_PITCH／GIMBAL_ROLL GPIO
constexpr U8 GIMBAL_GPIO[2] = {kUnassigned, kUnassigned};
constexpr U8 GIMBAL_LEDC_CH[2] = {0, 1};
constexpr U8 GIMBAL_LEDC_TIMER = 0;

}  // namespace ServoPwmPins
}  // namespace Components

#endif
