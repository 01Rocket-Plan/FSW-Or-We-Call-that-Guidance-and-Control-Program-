# Delivery: camera/esp32cam（DS ESP32-CAM）

## Commit / artifacts
- Branch: `camera/esp32cam` (worktree `/workspace/camera/review/esp32cam/repo`) — **not pushed**
- Base: `camera/ov3640` @ `7e5e8b12a1106edd30738a3016f198e794d48c4b` (from `/workspace/camera/deliver/ov3640/ov3640.bundle`)
- Bundle: `/workspace/camera/deliver/esp32cam.bundle`（requires base `7e5e8b1`）
- Patch series: `/workspace/camera/deliver/esp32cam.patch`（from `7e5e8b1`）

## FPP 宣告
- **Model:** `ESP32_CAM`
- **Role:** `DS_CAM`
- Telemetry `CamStatus` fields: `camState`, `fault`, `frameCount`, `model`, `role`
  - 依首席建議用 `camState`（FPP 保留字 `state` 不可用）。

## Merge order
1. Merge **`camera/ov3640`** first（shared `CamTypes`：`camState` + Fw.Buffer image port；本分支**不**修改 `Ov3640Driver`）。
2. Then merge **`camera/esp32cam`**. DsCam `MOD_DEPS` → `Components_Camera_Ov3640Driver`；CMake 先登記 Ov3640Driver 再 DsCam。

## What landed
- `Esp32CamCore` / `Esp32CamSimDevice`（host-tested）
- F´ wrappers: `Esp32Cam.cpp/.hpp`, `Esp32CamSim.cpp/.hpp` + `Esp32Cam.fpp`
- Commands: `POWER_ON`, `RESET`, `CAPTURE_START`, `CAPTURE_STOP`（非 READY 時 CAPTURE_* → `EXECUTION_ERROR`）
- Rate ports: `sched1Hz`, `sched100Hz` only（no 1 kHz）
- Params: `MISS_LIMIT=3`, `FRAME_LIMIT=20`, `I2C_CLOCK_HZ=400000`, `I2C_ADDR=0`, `I2C_PIN_SDA/SCL=255`（placeholders）
- 影像：本機 SD + 獨立 `IGroundPreviewLink`（不走 ICamLink／遙測）；無匿名陣列 port（對齊 Bot2 Fw.Buffer 規範）
- Packet proposal: `Components/Camera/CamLinkPacket.md`（not in RocketTypes）
- No SSS / gate / 2000 m/FTS / keys

## Verification
- `./Components/Camera/DsCam/run_fpp_check.sh` → **fpp-check OK**（stubs + CamTypes + Esp32Cam.fpp）
- `./Components/Camera/DsCam/run_host_tests.sh` → Esp32CamSimTest + WrapperLogic + Ov3640SimTest（`-Wall -Wextra -Werror`）
- **Full `fprime-util generate` / topology link（Ac headers）NOT verified.** 交**首席**於整合時執行。

## Remaining placeholders / open issues
1. I2C address / SDA / SCL still placeholders — Anatoly pin table.
2. CamLinkPacket SOF/length/CRC vs Sim bare type bytes — wire with I2C driver.
3. Ground preview PHY（WiFi vs RF）TBD；`IGroundPreviewLink` stubbed.
4. Wrapper `.cpp` needs generated Ac headers before deployment link（chief at integration）.
