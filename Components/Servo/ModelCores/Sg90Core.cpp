// =============================================================================
// Sg90Core.cpp
// =============================================================================
#include "Components/Servo/ModelCores/Sg90Core.hpp"

#include <cmath>

namespace Components {

uint16_t Sg90Core::clampPulseUs(uint16_t raw_us, bool& clamped) const {
  clamped = false;
  if (raw_us < Sg90Limits::kPulseMinUs) {
    clamped = true;
    return Sg90Limits::kPulseMinUs;
  }
  if (raw_us > Sg90Limits::kPulseMaxUs) {
    clamped = true;
    return Sg90Limits::kPulseMaxUs;
  }
  return raw_us;
}

float Sg90Core::pulseToAngleDeg(uint16_t pulse_us) const {
  bool dummy = false;
  const uint16_t p = clampPulseUs(pulse_us, dummy);
  const float span_us =
      static_cast<float>(Sg90Limits::kPulseMaxUs - Sg90Limits::kPulseMinUs);
  const float offset_us =
      static_cast<float>(static_cast<int>(p) - static_cast<int>(Sg90Limits::kNeutralUs));
  // Angle relative to neutral: full span maps to +/- kTravelDeg/2.
  return (offset_us / span_us) * Sg90Limits::kTravelDeg;
}

uint16_t Sg90Core::angleToPulseUs(float angle_deg) const {
  const float half_travel = 0.5f * Sg90Limits::kTravelDeg;
  if (angle_deg <= -half_travel) {
    return Sg90Limits::kPulseMinUs;
  }
  if (angle_deg >= half_travel) {
    return Sg90Limits::kPulseMaxUs;
  }
  const float span_us =
      static_cast<float>(Sg90Limits::kPulseMaxUs - Sg90Limits::kPulseMinUs);
  const float pulse = static_cast<float>(Sg90Limits::kNeutralUs) +
                      (angle_deg / Sg90Limits::kTravelDeg) * span_us;
  // Guard rounding edge cases against hard limits.
  if (pulse <= static_cast<float>(Sg90Limits::kPulseMinUs)) {
    return Sg90Limits::kPulseMinUs;
  }
  if (pulse >= static_cast<float>(Sg90Limits::kPulseMaxUs)) {
    return Sg90Limits::kPulseMaxUs;
  }
  return static_cast<uint16_t>(std::lround(pulse));
}

}  // namespace Components
