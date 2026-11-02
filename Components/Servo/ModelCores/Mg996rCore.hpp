// =============================================================================
// Mg996rCore.hpp
//
// Pure C++14 core for the MG996R model: pulse clamp / map helpers plus
// lock/fire convenience accessors. No F' / no ESP-IDF / no sleeps / no time.
//
// Separation fire-once / NV semantics live in SepServoDriver / SepSimCore,
// NOT here:
//   - after fire_hold_s the channel STAYS at fire_us (no return to lock);
//   - at boot, an already-released channel outputs lock_us and does not re-fire;
//   - NvFlags set failure → stay at lock_us, allow retry.
// This core only supplies pulse clamp/map and lock/fire pulse accessors.
// =============================================================================
#ifndef COMPONENTS_SERVO_MODELCORES_MG996RCORE_HPP_
#define COMPONENTS_SERVO_MODELCORES_MG996RCORE_HPP_

#include <cstdint>

namespace Components {

struct Mg996rLimits {
  static constexpr uint16_t kPulseMinUs = 1000;  // lock default
  static constexpr uint16_t kPulseMaxUs = 2000;  // fire default
  static constexpr uint16_t kNeutralUs = 1500;
  static constexpr uint16_t kPwmHz = 50;
  // Datasheet-typical; assumption, NOT measured on vehicle.
  static constexpr float kTravelDeg = 120.0f;
};

class Mg996rCore {
 public:
  Mg996rCore() = default;

  uint16_t clampPulseUs(uint16_t raw_us, bool& clamped) const;

  float pulseToAngleDeg(uint16_t pulse_us) const;
  uint16_t angleToPulseUs(float angle_deg) const;

  uint16_t lockPulseUs() const { return Mg996rLimits::kPulseMinUs; }
  uint16_t firePulseUs() const { return Mg996rLimits::kPulseMaxUs; }
  uint16_t neutralUs() const { return Mg996rLimits::kNeutralUs; }
  uint16_t pwmHz() const { return Mg996rLimits::kPwmHz; }
};

}  // namespace Components

#endif  // COMPONENTS_SERVO_MODELCORES_MG996RCORE_HPP_
