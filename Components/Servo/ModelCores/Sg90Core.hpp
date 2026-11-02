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
  // Datasheet-typical hobby servo travel; assumption, NOT measured on vehicle.
  static constexpr float kTravelDeg = 180.0f;
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
