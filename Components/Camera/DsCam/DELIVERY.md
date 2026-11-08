# Delivery: camera/esp32cam（DS ESP32-CAM）

## Commit / artifacts
- Branch: `camera/esp32cam` (worktree `/workspace/camera/review/esp32cam/repo`) — **not pushed**
- **HEAD:** `6e9674c442e6481e9216412bbd347fa50ae8ec15`
- **Base:** `camera/ov3640` @ `d66ec9b2ba47e36faeb4ef8ed1c5535e4685f603`
- Bundle: `/workspace/camera/deliver/esp32cam.bundle`（requires `d66ec9b2ba47e36faeb4ef8ed1c5535e4685f603`）
- Patch series: `/workspace/camera/deliver/esp32cam.patch`（from `d66ec9b2ba47e36faeb4ef8ed1c5535e4685f603`）

## Edward PR #14 review → Bot2 PR #12 @ d66ec9b
1. Fault 只回報無挽救 → Bot2：fault recovery K=5 / R=3（Ov3640Core）
2. 整張 JPEG vs 2KB 分段 → Bot2：CamChunkFlag + 2KB chunked `Fw.Buffer`；**DsCam 亦加 `Esp32CamImageOut` 同方案**
3. 截斷壞幀 → Bot2：`CamFault::FRAME_TRUNCATED`；DsCam：`enterFault(FRAME_TRUNCATED)`／截斷分塊 **僅事件、不進 FAULT、不計 frameCount、維持 READY**

## 2KB chunked imageOut（首席要求，對齊 Ov3640）
- Port `Esp32CamImageOut(frameNumber, chunkIndex, flags, ref data: Fw.Buffer)`；`Esp32Cam_FRAME_BUF_MAX=2048`
- 共用 `CamChunkFlag`；超過 `FRAME_MAX_BYTES` → 最後塊 `END|TRUNCATED`、不計 `frameCount`、維持 READY
- Host：`Esp32CamImageStreamer` + test 11（5000→3 chunks）／test 12（truncate then next counts）

## FPP 宣告
- **Model:** `ESP32_CAM`　**Role:** `DS_CAM`
- Telemetry `CamStatus`：`camState`, `fault`, `frameCount`, `model`, `role`（依首席建議用 `camState`）

## Merge order
1. Merge **`camera/ov3640` @ d66ec9b** first（已 `git merge`；**不** rebase）。
2. Then `camera/esp32cam`。不修改 `Ov3640Driver`／`CamTypes`。

## Verification
- `./Components/Camera/DsCam/run_fpp_check.sh` → fpp-check OK
- `./Components/Camera/DsCam/run_host_tests.sh` → Esp32CamSimTest 12/12 + WrapperLogic + Ov3640 Sim/Adapter
- **`fprime-util generate` / topology link NOT verified**（首席整合時執行）
