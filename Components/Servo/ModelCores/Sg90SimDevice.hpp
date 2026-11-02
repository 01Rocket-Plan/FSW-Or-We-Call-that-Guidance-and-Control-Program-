// =============================================================================
// Sg90SimDevice.hpp
//
// Device-level SG90 simulator: converts commanded pulse -> shaft angle with
// slew-rate limit. Pure math, no bus / GPIO. Used by ServoDriverSim so SIL
// telemetry shows motion (not just commanded us).
// =============================================================================
#ifndef COMPONENTS_SERVO_MODELCORES_SG90SIMDEVICE_HPP_
#define COMPONENTS_SERVO_MODELCORES_SG90SIMDEVICE_HPP_

#include <cstdint>

#include "Components/Servo/ModelCores/Sg90Core.hpp"

namespace Components {

class Sg90SimDevice {
 public:
  static constexpr uint8_t kMaxChannels = 4;
  // Datasheet-nominal (SG90 ~0.10 s / 60 deg -> ~600 deg/s). NOT measured
  // on vehicle; tunable via test/telemetry.
  static constexpr float kSlewDegPerSec = 600.0f;

  Sg90SimDevice();

  // Commanded pulse from driver (µs). Defensively re-clamped inside.
  void setCommandPulseUs(uint8_t ch, uint16_t pulse_us);

  // Advance simulation by dt_s. Caller derives dt from sched ticks
  // (e.g. 0.02 s @ 50 Hz). dt_s == 0 is a no-op.
  void tick(float dt_s);

  float angleDeg(uint8_t ch) const;
  uint16_t measuredPulseUs(uint8_t ch) const;  // angle -> pulse via Sg90Core

 private:
  Sg90Core core_;
  uint16_t commanded_us_[kMaxChannels];
  float angle_deg_[kMaxChannels];
};

}  // namespace Components

#endif  // COMPONENTS_SERVO_MODELCORES_SG90SIMDEVICE_HPP_
