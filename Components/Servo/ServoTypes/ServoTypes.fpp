@ Servo formal identity types (model / role / channel enums and per-model pulse constants).
@ Used by ServoDriver, SepServoDriver, and their Sim shells so the MCU/compiler knows
@ which physical servo each topology instance is (SG90 TAIL vs SG90 GIMBAL vs MG996R SEP).
module Components {

  @ Hardware model of the servo actuator
  enum ServoModel { SG90, MG996R }

  @ Deployment role of a driver instance
  enum ServoRole { TAIL, GIMBAL, SEP }

  @ DS tail fin channels (ServoDriver role=TAIL, channel_count=4)
  enum TailChannel { TAIL_1, TAIL_2, TAIL_3, TAIL_4 }

  @ US camera gimbal channels (ServoDriver role=GIMBAL, channel_count=2)
  enum GimbalChannel { PITCH, ROLL }

  @ DS interstage separation channels (SepServoDriver role=SEP)
  enum SepChannel { SEP_1, SEP_2, SEP_3, SEP_4 }

  @ SG90 pulse / rate constants (µs, Hz). Separate from MG996R even when numeric values match.
  constant SG90_PULSE_MIN_US = 1000
  constant SG90_PULSE_MAX_US = 2000
  constant SG90_NEUTRAL_US = 1500
  constant SG90_PWM_HZ = 50

  @ MG996R pulse / rate constants (µs, Hz). lock/fire use min/max; neutral kept for symmetry.
  constant MG996R_PULSE_MIN_US = 1000
  constant MG996R_PULSE_MAX_US = 2000
  constant MG996R_NEUTRAL_US = 1500
  constant MG996R_PWM_HZ = 50

}
