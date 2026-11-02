// =============================================================================
// Mg996rCore.cpp
// =============================================================================
#include "Components/Servo/ModelCores/Mg996rCore.hpp"

#include <cmath>

namespace Components {

uint16_t Mg996rCore::clampPulseUs(uint16_t raw_us, bool& clamped) const {
  clamped = false;
  if (raw_us < Mg996rLimits::kPulseMinUs) {
    clamped = true;
    return Mg996rLimits::kPulseMinUs;
  }
  if (raw_us > Mg996rLimits::kPulseMaxUs) {
    clamped = true;
    return Mg996rLimits::kPulseMaxUs;
  }
  return raw_us;
}

float Mg996rCore::pulseToAngleDeg(uint16_t pulse_us) const {
  bool dummy = false;
  const uint16_t p = clampPulseUs(pulse_us, dummy);
  const float span_us =
      static_cast<float>(Mg996rLimits::kPulseMaxUs - Mg996rLimits::kPulseMinUs);
  const float offset_us =
      static_cast<float>(static_cast<int>(p) - static_cast<int>(Mg996rLimits::kNeutralUs));
  return (offset_us / span_us) * Mg996rLimits::kTravelDeg;
}

uint16_t Mg996rCore::angleToPulseUs(float angle_deg) const {
  const float half_travel = 0.5f * Mg996rLimits::kTravelDeg;
  if (angle_deg <= -half_travel) {
    return Mg996rLimits::kPulseMinUs;
  }
  if (angle_deg >= half_travel) {
    return Mg996rLimits::kPulseMaxUs;
  }
  const float span_us =
      static_cast<float>(Mg996rLimits::kPulseMaxUs - Mg996rLimits::kPulseMinUs);
  const float pulse = static_cast<float>(Mg996rLimits::kNeutralUs) +
                      (angle_deg / Mg996rLimits::kTravelDeg) * span_us;
  if (pulse <= static_cast<float>(Mg996rLimits::kPulseMinUs)) {
    return Mg996rLimits::kPulseMinUs;
  }
  if (pulse >= static_cast<float>(Mg996rLimits::kPulseMaxUs)) {
    return Mg996rLimits::kPulseMaxUs;
  }
  return static_cast<uint16_t>(std::lround(pulse));
}

}  // namespace Components
