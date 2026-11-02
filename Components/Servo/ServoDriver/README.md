# ServoDriver

- Owner: servo group (Evgeny)
- Formal identity: params `model`/`role` from `Components/Servo/ServoTypes` — allowed `(SG90,TAIL,4)` or `(SG90,GIMBAL,2)`; else SELFTEST FAULT + `ModelRoleMismatch`
- Instances: DS tail `servoDriver` 0x3000; US gimbal `gimbalServoDriver` 0x3200
- PWM: `Components/Servo/Pwm` (pins in `ServoPwmPins.hpp`, MCU v0.1 draft)
- Topology proposal: `Components/Servo/INTEGRATION.md`
- Per-model cores (Sg90Core) via DeepSeek spec, not hand-written in this commit
