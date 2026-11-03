# Delivery: camera/esp32cam（DS ESP32-CAM）

## Commit / artifacts
- Branch: `camera/esp32cam` (worktree `/workspace/camera/review/esp32cam/repo`) — **not pushed**
- Bundle: `/workspace/camera/deliver/esp32cam.bundle`
- Patch: `/workspace/camera/deliver/esp32cam.patch`

## FPP 宣告
- **Model:** `ESP32_CAM`
- **Role:** `DS_CAM`
- Telemetry `CamStatus` fields (**Arthur finalized**): `camState`, `fault`, `frameCount`, `model`, `role`
  - `camState` replaces reserved FPP word `state`.

## Merge order
1. Merge **`camera/ov3640`** first (shared `Components/Camera/Ov3640Driver/CamTypes`, including `camState` + Fw.Buffer image port).
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
- `./Components/Camera/DsCam/run_host_tests.sh` → Esp32CamSimTest + WrapperLogic + Ov3640SimTest (`-Wall -Wextra -Werror`)
- **Full `fprime-util generate` / topology link (needs Ac headers) NOT verified.** To be run by **chief engineer at integration**.

## Remaining placeholders / open issues
1. I2C address / SDA / SCL still placeholders — Anatoly pin table.
2. CamLinkPacket SOF/length/CRC vs Sim bare type bytes — wire with I2C driver.
3. Ground preview PHY (WiFi vs RF) TBD; `IGroundPreviewLink` stubbed.
4. Wrapper `.cpp` needs generated Ac headers before deployment link (chief at integration).
