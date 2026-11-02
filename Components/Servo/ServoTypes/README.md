# ServoTypes

Formal FPP identity for servo driver instances:

- `ServoModel` — `SG90` / `MG996R`
- `ServoRole` — `TAIL` / `GIMBAL` / `SEP`
- Channel enums — `TailChannel`, `GimbalChannel`, `SepChannel`
- Per-model pulse / PWM constants (`SG90_*`, `MG996R_*`)

Drivers take `model` / `role` params and FAULT on mismatch in SELFTEST.
Board pins stay in `Components/Servo/Pwm/ServoPwmPins.hpp`.
Topology wiring proposal: `Components/Servo/INTEGRATION.md`.
