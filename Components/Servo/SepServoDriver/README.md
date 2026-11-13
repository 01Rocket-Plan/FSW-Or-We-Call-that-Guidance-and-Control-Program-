# SepServoDriver

- Owner: servo group (Evgeny)
- 通道：**3 顆**（SEP_1–3，環排 120°，SSS §7；Edward 2026-11-10）
- Formal identity: `model=MG996R`, `role=SEP` (ServoTypes); mismatch → FAULT + `ModelRoleMismatch`
- Instance: DS `sepServoDriver` 0x3100; fire-once via `Power.SepReleasedGet`/`Set`
- PWM: shared `Components/Servo/Pwm`; SEP pins `ServoPwmPins::SEP_*`
- Topology proposal: `Components/Servo/INTEGRATION.md`
- Per-model core (Mg996rCore) via DeepSeek spec
