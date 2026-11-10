# Servo topology integration (proposal — Deployments/ not edited yet)

> Author: Evgeny (servo group) | 2026-11-03 Asia/Taipei
> Status: **proposal for chief engineer**. Do **not** merge topology changes until approved.
> Formal identity: `Components/Servo/ServoTypes/ServoTypes.fpp` (`ServoModel` / `ServoRole`).

Board pins remain in `Components/Servo/Pwm/ServoPwmPins.hpp` (MCU group v0.1 draft).

## Instance table

| Instance | Component | Board | Base ID | Rate group | `model` | `role` | `channel_count` |
|---|---|---|---|---|---|---|---|
| `servoDriver` | `Components.ServoDriver` | DsMcu | `0x3000` | rg1kHz | `SG90` | `TAIL` | `4` |
| `sepServoDriver` | `Components.SepServoDriver` | DsMcu | `0x3100` | rg1kHz | `MG996R` | `SEP` | (fixed 3) |
| `gimbalServoDriver` | `Components.ServoDriver` | UsMcu | `0x3200` | rg1kHz | `SG90` | `GIMBAL` | `2` |
| `servoDriverSim` | `Components.ServoDriverSim` | DsMcu Sim | `0x3080` | rg1kHz | `SG90` | `TAIL` | `4` |
| `sepServoDriverSim` | `Components.SepServoDriverSim` | DsMcu Sim | `0x3180` | rg1kHz | `MG996R` | `SEP` | (fixed 3) |
| `gimbalServoDriverSim` | `Components.ServoDriverSim` | UsMcu Sim | `0x3280` | rg1kHz | `SG90` | `GIMBAL` | `2` |

Sim base IDs = real + `0x80` (conventions).

## Connections (real drivers)

### DsMcu — `servoDriver` (TAIL)

| From | Port | To | Port |
|---|---|---|---|
| rate group rg1kHz | `RateGroupMemberOut[n]` | `servoDriver` | `schedIn` |
| `actuatorMgr` | servo cmd out (tail) | `servoDriver` | `cmdIn` |
| `powerMonitor` / UV source | `UvState` out | `servoDriver` | `uvIn` |
| (optional SIL) | — | `servoDriver` | `pwmOut` → Sim observer |
| F´ services | time / cmd / tlm / event / param | `servoDriver` | `timeCaller`, `cmdDispIn`, `cmdRegOut`, `cmdResponseOut`, `tlmOut`, `eventOut`, `textEventOut`, `prmGetOut`, `prmSetOut` |

`configure(...)` at topology init: TAIL GPIO `{25,26,27,14}`, LEDC HS ch `{0,1,2,3}`, timer `0` (`ServoPwmPins::TAIL_*`).

### DsMcu — `sepServoDriver` (SEP)

| From | Port | To | Port |
|---|---|---|---|
| rg1kHz | member | `sepServoDriver` | `schedIn` |
| `actuatorMgr` | sep cmd out | `sepServoDriver` | `cmdIn` |
| UV source | `UvState` | `sepServoDriver` | `uvIn` |
| `sepServoDriver` | `sepReleasedGet` | `nvFlags` | `sepReleasedGet` (`Power.SepReleasedGet`) |
| `sepServoDriver` | `sepReleasedSet` | `nvFlags` | `sepReleasedSet` (`Power.SepReleasedSet`) |
| F´ services | (same set as above) | `sepServoDriver` | (same) |

`configure(...)`: SEP GPIO `{32,33,13}`, LEDC HS ch `{4,5,6}`, timer `0`（3×MG996R 環排 120°，SSS §7；2026-11-10 由 4 顆改 3 顆）.

### UsMcu — `gimbalServoDriver` (GIMBAL)

Same pattern as tail `servoDriver`, but:

- Base ID `0x3200`, params `role=GIMBAL`, `channel_count=2`
- GPIO `{25,26}` (PITCH/ROLL), LEDC HS ch `{0,1}`, timer `0`
- `actuatorMgr` (US) → `cmdIn` with `pulse_us[0]`=PITCH, `[1]`=ROLL; `[2]`/`[3]` ignored by driver

## Connections (Sim)

Mirror the real wiring. Prefer Sim instances in SIL topologies:

- `pwmOut` **must** be connected to a recorder / SIL harness so commanded µs (and later SimDevice angles) are visible.
- `sepReleasedGet` / `sepReleasedSet` → `nvFlags` Sim or a test double implementing `Power.SepReleased*`.

## Param defaults vs topology overrides

FPP defaults already match the primary instance (`ServoDriver` → SG90/TAIL/4; `SepServoDriver` → MG996R/SEP). Topology **must** override for gimbal:

```
gimbalServoDriver.role = GIMBAL
gimbalServoDriver.channel_count = 2
```

SELFTEST FAULTs with `ModelRoleMismatch` + `EnterFault(reason=3)` if `(model,role,channel_count)` is not an allowed combo.

## Out of scope here

- Editing any file under `Deployments/` (pending chief approval).
- `Sg90Core` / `Mg996rCore` / SimDevices — requirements for DeepSeek: `/workspace/servo/deepseek_req_servo_cores_v1.md`.
