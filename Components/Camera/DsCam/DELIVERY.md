# Delivery: camera/esp32cam（DS ESP32-CAM）

## Commit
- `91fb75e2293cbfcffab30bff2f0299eec022d885` (`91fb75e`) on `camera/esp32cam` (base `camera/ov3640` @ `0b7ef2b`)
- Bundle: `/workspace/camera/deliver/esp32cam.bundle`
- Patch: `/workspace/camera/deliver/esp32cam.patch` (same content as `0001-DS-ESP32-CAM-Core-Sim-F-ov3640-CamStatus.hsState.patch`)
- Worktree: `/workspace/camera/review/esp32cam/repo` — **not pushed**

## FPP 宣告
- **Model:** `ESP32_CAM`
- **Role:** `DS_CAM`
- Telemetry `CamStatus` fields: `hsState`, `fault`, `frameCount`, `model`, `role`
  - Assumed rename: FPP reserved word `state` → **`hsState`** (BaroDriver `HsState` style).
  - **Rebase pending:** `camera/ov3640` had no commit after `0b7ef2b` with CamTypes fix at delivery. If 子Bot2 picks another name, rebase and adapt.

## Merge order
1. Merge **`camera/ov3640`** first (shared `Components/Camera/Ov3640Driver/CamTypes`).
2. Then merge **`camera/esp32cam`**. DsCam `MOD_DEPS` → `Components_Camera_Ov3640Driver`; CMake registers Ov3640Driver before DsCam.

## What landed
- `Esp32CamCore` / `Esp32CamSimDevice` (host-tested)
- F´ wrappers: `Esp32Cam.cpp/.hpp`, `Esp32CamSim.cpp/.hpp` + `Esp32Cam.fpp`
- Commands: `POWER_ON`, `RESET`, `CAPTURE_START`, `CAPTURE_STOP` (CAPTURE_* when not READY → `EXECUTION_ERROR`)
- Rate ports: `sched1Hz`, `sched100Hz` only (no 1 kHz)
- Params: `MISS_LIMIT=3`, `FRAME_LIMIT=20`, `I2C_CLOCK_HZ=400000`, `I2C_ADDR=0`, `I2C_PIN_SDA/SCL=255` (placeholders)
- Packet proposal: `Components/Camera/CamLinkPacket.md` (not in RocketTypes)
- No SSS / gate / 2000 m/FTS / keys

## Verification
- `./Components/Camera/DsCam/run_fpp_check.sh` → **fpp-check OK** (stubs + CamTypes + Esp32Cam.fpp)
- `./Components/Camera/DsCam/run_host_tests.sh` → Esp32CamSimTest 9/9, WrapperLogic passed, Ov3640SimTest 10/10 (`-Wall -Wextra -Werror`)
- Full `fprime-util generate` / topology link **not** run (no build cache here); `*ComponentAc.hpp` appears after generate

## Remaining placeholders / open issues
1. I2C address / SDA / SCL still placeholders — Anatoly pin table.
2. CamLinkPacket SOF/length/CRC vs Sim bare type bytes — wire with I2C driver.
3. Ground preview PHY (WiFi vs RF) TBD; `IGroundPreviewLink` stubbed.
4. Rebase onto Bot2 CamTypes fix when published (may differ from assumed `hsState`).
5. Ov3640Driver.fpp on `0b7ef2b` still has illegal `data: [N] U8` — Bot2; Esp32Cam.fpp has no inline array ports.
6. Wrapper `.cpp` needs generated Ac headers before deployment link.
