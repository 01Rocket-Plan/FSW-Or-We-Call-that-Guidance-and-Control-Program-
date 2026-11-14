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

SIL 設定（`Top/UsCamTopology.cpp`）：開機自動上電＋開始擷取；模擬感測器 10 fps、每幀 QCIF YUYV 50,688 B；
驅動只轉存 **1 fps**、只送 **灰階（Y）25,344 B＝13 塊**。

## 跑 SIL
```bash
tools/verify/fprime_verify.sh --run 5 Components/Camera/CamTypes Components/Camera/Ov7670Driver \
  Components/Recorder/SdLogger Deployments/UsCam
```
預期看到：`OFF -> INIT -> SELFTEST -> READY`（`StateChanged`），之後每秒轉存 1 幀（遙測 `Status.frameCount`）。
注意：SIL 沒有自動對 `sdLogger` 下 `START`，要從 GDS 下 `sdLogger.START` 影像才會寫進 SD；寫進去後每幀會有
`ImageChunkDrop`（佇列 8 塊、一幀 13 塊，見下；這個掉塊數是照程式算的，SIL 還沒實際量過）。
接 GDS：`fprime-gds` 開好後 `./Deployments_UsCam -a 127.0.0.1 -p 50000`，可以下 `CAPTURE_STOP`、`SIM_INJECT` 等指令。

## 已定（Edward 2026-11-14，PR #55）
- **上級 1 fps、144p＋灰階：** 感測器 QCIF 176×144 YUV422 → 驅動只取 Y → 一幀 25,344 B（13 塊 × 2 KB）。
  比原本 QVGA RGB565（153,600 B、75 塊）小 6 倍；1 fps 約 25 KB/s。
- **不做分批送、不改 SdLogger 佇列。** `SdLogger` 影像佇列預設 8 塊，一幀 13 塊 → **每幀仍會掉 5 塊**（`ImageChunkDrop`）。
  相機遙測是 P4 最低優先，「理論能傳、實則不行」，已知、接受。
- 分支 `camera/EdwardWang-Claude/ov3640-alt` 的 OV3640 不改（它能出 JPEG）。

## 還沒定
- 雲台 2×SG90 在 UsMcu（`gimbalServoDriver` 0x3200），不在這塊板子。
- CamLink（UsMcu ↔ 相機板）指令鏈路、腳位：ESP32 平台 PR 的全箭腳位表。
