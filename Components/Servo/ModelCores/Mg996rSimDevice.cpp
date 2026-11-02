// =============================================================================
// Mg996rSimDevice.cpp
// =============================================================================
#include "Components/Servo/ModelCores/Mg996rSimDevice.hpp"

#include <cmath>

namespace Components {

Mg996rSimDevice::Mg996rSimDevice() : core_() {
  for (uint8_t ch = 0; ch < kMaxChannels; ++ch) {
    commanded_us_[ch] = Mg996rLimits::kNeutralUs;
    angle_deg_[ch] = 0.0f;  // neutral
  }
}

void Mg996rSimDevice::setCommandPulseUs(uint8_t ch, uint16_t pulse_us) {
  if (ch >= kMaxChannels) {
    return;
  }
  // Defensive re-clamp even if caller already clamped (units: µs).
  // SEP lock/fire policy lives in SepServoDriver / SepSimCore, not here.
  bool clamped = false;
  commanded_us_[ch] = core_.clampPulseUs(pulse_us, clamped);
  (void)clamped;
}

void Mg996rSimDevice::tick(float dt_s) {
  if (!(dt_s > 0.0f)) {
    return;
  }
  const float max_step = kSlewDegPerSec * dt_s;
  for (uint8_t ch = 0; ch < kMaxChannels; ++ch) {
    const float target = core_.pulseToAngleDeg(commanded_us_[ch]);
    const float delta = target - angle_deg_[ch];
    if (std::fabs(delta) <= max_step) {
      angle_deg_[ch] = target;
    } else {
      angle_deg_[ch] += (delta > 0.0f) ? max_step : -max_step;
    }
  }
}

float Mg996rSimDevice::angleDeg(uint8_t ch) const {
  if (ch >= kMaxChannels) {
    return 0.0f;
  }
  return angle_deg_[ch];
}

uint16_t Mg996rSimDevice::measuredPulseUs(uint8_t ch) const {
  if (ch >= kMaxChannels) {
    return Mg996rLimits::kNeutralUs;
  }
  return core_.angleToPulseUs(angle_deg_[ch]);
}

}  // namespace Components
