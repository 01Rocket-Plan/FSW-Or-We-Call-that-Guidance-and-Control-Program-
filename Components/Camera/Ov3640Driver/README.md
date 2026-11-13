# Ov3640Driver（備援）

> **備援方案，不合 main。** 主案是 OV7670（main 的 `Components/Camera/Ov7670Driver`）。
> 這條分支 `camera/EdwardWang-Claude/ov3640-alt` = main（OV7670 版）把上級相機換成 OV3640；要換時從這條分支開 PR 即可。
> 換的內容：`Components/Camera/Ov7670Driver` → `Ov3640Driver`、`Components/CMakeLists.txt`、`Deployments/UsCam/Top`、`CamTypes.fpp` 的 `CamModel` 多一個 `OV3640 = 2`（原有值不變）。
> OV3640 有 JPEG，影像走同一個 `CamImageOut` 2 KB 分塊埠，SdLogger 不用改。

- 負責：相機驅動組子Bot2（US OV3640）
- 範圍：`Ov3640Core` + `Ov3640SimDevice` + F´ 外殼 `Ov3640Driver`／`Ov3640DriverSim` + FPP
- FPP 宣告：型號 `OV3640`、角色 `US_CAM`
- 遙測 `CamStatus` 欄位：`camState`、`fault`、`frameCount`、`model`、`role`（不變）
- 型別：`Components/Camera/CamTypes`（FPP 自動碼；`CamFrameView` 是唯一的 C++ 型別）
- 影像埠：`CamImageOut(frameNumber, chunkIndex, flags, ref data: Fw.Buffer)` → sdLogger
- Rate group：`sched1Hz`／`sched100Hz`；不接 1 kHz；無 sleep／系統時間／動態配置
- 測試：F´ UT `test/Ov3640SimTest.cpp`（gtest，13 項）：`tools/verify/fprime_verify.sh --ut`
- 原本靠手寫假自動碼（`test/host_ac`）的 AdapterTest 和 `fpp_stubs/` 已移除（真 F´ 規則，同 OV7670 版）

## 影像路徑（PR#14 binding）
- **2KB 分塊串流**（`Ov3640_FRAME_BUF_MAX`=2048）：每次 `imageOut` 只持有一個 chunk scratch，不在驅動內保留整張 JPEG。
- 每個 chunk 帶 `frameNumber` + `chunkIndex`；最後一塊 `flags` 含 `CamChunkFlag::END`。
- `FRAME_MAX_BYTES`（預設 65536）：超過則丟棄剩餘、最後一塊加 `TRUNCATED`、發 `FrameTruncated`；**該幀不計入 frameCount**。

## 故障恢復（PR#14 binding）
- FAULT 後於 **1 Hz** 每 `FAULT_RECOVER_K`（預設 5）tick 自動重跑 INIT，最多 `FAULT_RECOVER_R`（預設 3）次；每次發 `RecoverAttempt`。
- 超過 R 次後 **latch** FAULT；僅 `RESET` 可清除。
