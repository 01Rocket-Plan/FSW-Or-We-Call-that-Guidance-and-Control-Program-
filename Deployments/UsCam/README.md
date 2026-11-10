# UsCamDeployment（上級相機獨立板）

上級相機 **OV7670** 的獨立板（SSS §7），**不併入** `UsMcu` 主幹（CONTRIBUTING 第 9 條）。
本版是 **Linux SIL**（F´ v4.4.1）；ESP32 版跟 MCU 組 ESP32 平台 PR 一起做。

## 拓樸
| 實例 | 元件 | Base ID | 速率 |
|---|---|---|---|
| `usCam` | `Components.Ov7670DriverSim`（真機換 `Ov7670Driver` 0x7100） | 0x7180 | `sched1Hz`、`sched100Hz` |
| `sdLogger` | `Components.SdLoggerSim`（相機板本機 SD） | 0x2080 | 100 Hz |
| `rg100Hz`／`rg1Hz` | `Svc.ActiveRateGroup` | 0x0200／0x0300 | — |
| CdhCore／ComCcsds／FileHandling | F´ 子拓樸（指令、事件、遙測、參數、GDS） | F´ 預設 | 1 Hz |

連線：`usCam.imageOut`（2 KB 分塊，`Components.CamImageOut`）→ `sdLogger.usImageIn`。

SIL 設定（`Top/UsCamTopology.cpp`）：開機自動上電＋開始擷取、10 fps、每幀 16 KB。

## 跑 SIL
```bash
tools/verify/fprime_verify.sh --run 5 Components/Camera/CamTypes Components/Camera/Ov7670Driver \
  Components/Recorder/SdLogger Deployments/UsCam
```
預期看到：`OFF -> INIT -> SELFTEST -> READY`（`StateChanged`），之後每秒約 10 幀進 SD（遙測 `Status.frameCount`）。
接 GDS：`fprime-gds` 開好後 `./Deployments_UsCam -a 127.0.0.1 -p 50000`，可以下 `CAPTURE_STOP`、`SIM_INJECT` 等指令。

## 還沒定（要相機組＋紀錄組一起決定）
- **幀大小 vs SD 佇列：** OV7670 沒有 JPEG，一整張 QVGA RGB565 是 153,600 B（75 塊）；`SdLogger` 影像佇列預設深度 8 塊，
  整張丟進去會掉 67 塊。選項：佇列加深（RAM）、相機分批送（每拍 ≤8 塊）、或降解析度／改灰階。SIL 先用 16 KB／幀。
- 雲台 2×SG90 在 UsMcu（`gimbalServoDriver` 0x3200），不在這塊板子。
- CamLink（UsMcu ↔ 相機板）指令鏈路、腳位：ESP32 平台 PR 的全箭腳位表。
