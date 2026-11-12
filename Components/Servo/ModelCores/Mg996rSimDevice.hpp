// =============================================================================
// Mg996rSimDevice.hpp
//
// Device-level MG996R simulator: commanded pulse -> shaft angle with slew.
// Channels: 4 slots (generic); SepServoDriverSim uses SEP_1..3 only. Lock/fire pulse values still come from the driver;
// SimDevice only follows the commanded pulse.
// =============================================================================
#ifndef COMPONENTS_SERVO_MODELCORES_MG996RSIMDEVICE_HPP_
#define COMPONENTS_SERVO_MODELCORES_MG996RSIMDEVICE_HPP_

#include <cstdint>

#include "Components/Servo/ModelCores/Mg996rCore.hpp"

namespace Components {

class Mg996rSimDevice {
 public:
  static constexpr uint8_t kMaxChannels = 4;
  // Datasheet-nominal (MG996R ~0.17 s / 60 deg -> ~353 deg/s). NOT measured
  // on vehicle; document assumption.
  static constexpr float kSlewDegPerSec = 353.0f;

  Mg996rSimDevice();

  // Commanded pulse (µs). Defensively re-clamped inside.
  void setCommandPulseUs(uint8_t ch, uint16_t pulse_us);
  void tick(float dt_s);

  float angleDeg(uint8_t ch) const;
  uint16_t measuredPulseUs(uint8_t ch) const;

 private:
  Mg996rCore core_;
  uint16_t commanded_us_[kMaxChannels];
  float angle_deg_[kMaxChannels];
};

}  // namespace Components

#endif  // COMPONENTS_SERVO_MODELCORES_MG996RSIMDEVICE_HPP_
