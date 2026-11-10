// =============================================================================
// Sg90Core.hpp
//
// Pure C++14 core for the SG90 model: pulse clamp / map helpers only.
// No F' / no ESP-IDF / no sleeps / no system time / no dynamic allocation.
// Threading: single-threaded; caller serializes calls.
//
// Angle convention: 0 deg == neutral (kNeutralUs), linear map across
// [kPulseMinUs, kPulseMaxUs] over [-kTravelDeg/2, +kTravelDeg/2].
// =============================================================================
#ifndef COMPONENTS_SERVO_MODELCORES_SG90CORE_HPP_
#define COMPONENTS_SERVO_MODELCORES_SG90CORE_HPP_

#include <cstdint>

namespace Components {

// Numeric values MUST match FPP ServoTypes / servo spec sheet.
struct Sg90Limits {
  static constexpr uint16_t kPulseMinUs = 1000;
  static constexpr uint16_t kPulseMaxUs = 2000;
  static constexpr uint16_t kNeutralUs = 1500;
  static constexpr uint16_t kPwmHz = 50;
  // 行程 360°（Edward 2026-11-10 #38／#42 B3：買的 SG90、MG996R 都是 360° 版）。
  // 脈寬窗仍沿用 1000–2000 µs，等規格書／冷測確認；量到再改這裡和 ActuatorMgr usPerRad。
  // ⚠ 多數「360° 舵機」是連續旋轉（脈寬控轉速、不能定位）；若是那種，尾翼／雲台／分離都不能用。
  static constexpr float kTravelDeg = 360.0f;
};

class Sg90Core {
 public:
  Sg90Core() = default;

  // Clamp raw command into [kPulseMinUs, kPulseMaxUs].
  // Sets clamped=true iff the input was out of range.
  uint16_t clampPulseUs(uint16_t raw_us, bool& clamped) const;

  // Linear map pulse -> angle. Input pulse is clamped first.
  float pulseToAngleDeg(uint16_t pulse_us) const;

  // Linear map angle -> pulse. Angle saturates at +/- kTravelDeg/2.
  uint16_t angleToPulseUs(float angle_deg) const;

  uint16_t neutralUs() const { return Sg90Limits::kNeutralUs; }
  uint16_t pwmHz() const { return Sg90Limits::kPwmHz; }
};

}  // namespace Components

#endif  // COMPONENTS_SERVO_MODELCORES_SG90CORE_HPP_
