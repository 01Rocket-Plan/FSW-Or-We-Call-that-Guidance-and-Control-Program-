# Servo Model Cores

Pure C++14 host-testable cores + simulators for SG90 and MG996R actuators.
No F´, no ESP-IDF, no sleeps, no wall-clock, no dynamic allocation in
steady state. Constants match `ServoTypes` (FPP identity).

## Contents

| File | Responsibility |
|---|---|
| `Sg90Core.{hpp,cpp}`       | SG90 pulse clamp / map (1000–2000 µs, 50 Hz, **360°**) |
| `Mg996rCore.{hpp,cpp}`     | MG996R pulse clamp / map (1000–2000 µs, 50 Hz, **360°**) + lock/fire accessors |
| `Sg90SimDevice.{hpp,cpp}`  | 4-channel SG90 slew simulator |
| `Mg996rSimDevice.{hpp,cpp}`| 4-channel MG996R slew simulator |

## 行程（2026-11-10 更新）

Edward（#38、#42 B3）：實際買的 SG90、MG996R **都是 360° 版**，`kTravelDeg` 由 180°／120° 改為 360°。
脈寬窗暫沿用 1000–2000 µs（舊值），等規格書或冷測確認；改了要同步 `ActuatorMgr` 的 `usPerRad`
（360° / 1000 µs → 159.15 µs/rad；尾翼 ±30° = ±83 µs）。

> ⚠ 市面上「360° 舵機」多半是**連續旋轉**版（脈寬控制轉速、沒有位置回授），那種不能拿來定位尾翼、
> 雲台或分離擋板。冷測時送 1500 µs → 1600 µs：停在新角度＝定位型（OK）；一直轉＝連續旋轉（要換料）。

## Conventions

- Pulse units: microseconds (`uint16_t`).
- Angle units: degrees (`float`), `0° == neutral` (1500 µs).
- Angle maps linearly: pulse span `[min,max]` ⇔ `[-kTravelDeg/2, +kTravelDeg/2]`.
- Time advances **only** via `tick(dt_s)` — no `sleep`, no `std::chrono`.

## Slew assumptions (datasheet-nominal, NOT vehicle-measured)

| Model   | Datasheet spec    | `kSlewDegPerSec` |
|---------|-------------------|------------------|
| SG90    | ~0.10 s / 60°     | **600**          |
| MG996R  | ~0.17 s / 60°     | **353**          |

These are nominal datasheet figures at moderate load and are explicitly
labelled as assumptions so SIL telemetry is causally reasonable — not a
measurement of any specific unit. Any vehicle-specific performance number
must not be baked in here.

## Non-goals

- LEDC/timer PWM backends (see `Components/Servo/Pwm`).
- NvFlags / fire-once policy (see `SepServoDriver` / `SepSimCore`).
- Topology wiring (see `INTEGRATION.md`).


## Integration (this tree)

- Registered as F´ module `Components_Servo_ModelCores` via `Components/CMakeLists.txt`.
- `ServoDriver` / `SepServoDriver` call `Sg90Core` / `Mg996rCore` for model hard-limit clamp (then optional tighter param window). Board pins stay in `Pwm/ServoPwmPins.hpp`.
- `ServoDriverSim` / `SepServoDriverSim` feed commanded µs into `Sg90SimDevice` / `Mg996rSimDevice` and `tick(0.02)` each PWM period so SIL can observe shaft angle (telemetry names deferred).
- Host: `CXX="python3 -m ziglang c++" bash Components/Servo/run_sim_host_tests.sh`

## Build integration (suggested)

CMake target `Components_Servo_ModelCores`:

```cmake
add_library(Components_Servo_ModelCores STATIC
  Sg90Core.cpp
  Mg996rCore.cpp
  Sg90SimDevice.cpp
  Mg996rSimDevice.cpp
)
target_include_directories(Components_Servo_ModelCores PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
target_compile_features(Components_Servo_ModelCores PUBLIC cxx_std_14)
target_compile_options(Components_Servo_ModelCores PRIVATE -Wall -Wextra -Werror)
```

`MOD_DEPS` for the drivers/Sims (`ServoDriver`, `SepServoDriver` and their
Sim shells) should list `Components_Servo_ModelCores`.

Host test target (not in F´ build):

```cmake
add_executable(ModelCoresHostTests test/ut/ModelCoresTest.cpp)
target_link_libraries(ModelCoresHostTests PRIVATE Components_Servo_ModelCores)
target_compile_options(ModelCoresHostTests PRIVATE -std=c++14 -Wall -Wextra -Werror)
```

## Acceptance

`test/ut/ModelCoresTest.cpp` exercises:

1. Clamp (900→1000, 2100→2000, mid unchanged, flag correct) — both cores.
2. Pulse/angle round-trip at neutral, min, max; saturation outside travel.
3. SG90 slew: 30° after 0.05 s (±2°); reaches +180° end-stop and holds.
4. MG996R slew: ~17.65° after 0.05 s (±2°); reaches +180° end-stop and holds.
5. `dt = 0` (and negative) is a no-op.
6. Channels are independent.
7. Out-of-range channel index is ignored.
8. Static review: implementation contains no `sleep`, no `std::chrono`,
   no `<thread>`, no dynamic allocation.
