// =============================================================================
// Sg90SimDevice.cpp
// =============================================================================
#include "Components/Servo/ModelCores/Sg90SimDevice.hpp"

#include <cmath>

namespace Components {

Sg90SimDevice::Sg90SimDevice() : core_() {
  for (uint8_t ch = 0; ch < kMaxChannels; ++ch) {
    commanded_us_[ch] = Sg90Limits::kNeutralUs;
    angle_deg_[ch] = 0.0f;  // neutral
  }
}

void Sg90SimDevice::setCommandPulseUs(uint8_t ch, uint16_t pulse_us) {
  if (ch >= kMaxChannels) {
    return;
  }
  // Defensive re-clamp even if caller already clamped (units: µs).
  bool clamped = false;
  commanded_us_[ch] = core_.clampPulseUs(pulse_us, clamped);
  (void)clamped;
}

void Sg90SimDevice::tick(float dt_s) {
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

float Sg90SimDevice::angleDeg(uint8_t ch) const {
  if (ch >= kMaxChannels) {
    return 0.0f;
  }
  return angle_deg_[ch];
}

uint16_t Sg90SimDevice::measuredPulseUs(uint8_t ch) const {
  if (ch >= kMaxChannels) {
    return Sg90Limits::kNeutralUs;
  }
  return core_.angleToPulseUs(angle_deg_[ch]);
}

}  // namespace Components
