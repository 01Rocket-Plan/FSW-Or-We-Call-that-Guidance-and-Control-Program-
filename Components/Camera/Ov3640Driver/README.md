# Ov3640Driver

- 負責：相機驅動組子Bot2（US OV3640）
- 範圍：`Ov3640Core` + `Ov3640SimDevice` + F´ 外殼 `Ov3640Driver`／`Ov3640DriverSim` + FPP
- FPP 宣告：型號 `OV3640`、角色 `US_CAM`
- 遙測 `CamStatus` 欄位：`camState`、`fault`、`frameCount`、`model`、`role`（不變）
- 影像埠：`Ov3640ImageOut(frameNumber, chunkIndex, flags, ref data: Fw.Buffer)` → sdLogger
- Rate group：`sched1Hz`／`sched100Hz`；不接 1 kHz；無 sleep／系統時間／動態配置
- Host：`./run_host_tests.sh`（含 `fpp-check` + Sim／Adapter 測試）
- `fpp_stubs/` 僅供本機 fpp-check，不進飛行映像

## 影像路徑（PR#14 binding）
- **2KB 分塊串流**（`Ov3640_FRAME_BUF_MAX`=2048）：每次 `imageOut` 只持有一個 chunk scratch，不在驅動內保留整張 JPEG。
- 每個 chunk 帶 `frameNumber` + `chunkIndex`；最後一塊 `flags` 含 `CamChunkFlag::END`。
- `FRAME_MAX_BYTES`（預設 65536）：超過則丟棄剩餘、最後一塊加 `TRUNCATED`、發 `FrameTruncated`；**該幀不計入 frameCount**。

## 故障恢復（PR#14 binding）
- FAULT 後於 **1 Hz** 每 `FAULT_RECOVER_K`（預設 5）tick 自動重跑 INIT，最多 `FAULT_RECOVER_R`（預設 3）次；每次發 `RecoverAttempt`。
- 超過 R 次後 **latch** FAULT；僅 `RESET` 可清除。
