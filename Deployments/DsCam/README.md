# DsCamDeployment（下級相機獨立板）

下級相機 **ESP32-CAM** 的獨立板（SSS §7），**不併入** `DsMcu` 主幹（CONTRIBUTING 第 9 條）。
本版是 **Linux SIL**（F´ v4.4.1），拓樸照 `Deployments/UsCam`；ESP32 版跟 MCU 組 ESP32 平台 PR 一起做。

## 拓樸
| 實例 | 元件 | Base ID | 速率 |
|---|---|---|---|
| `dsCam` | `Components.Esp32CamSim`（真機換 `Esp32Cam` 0x7000） | 0x7080 | `sched1Hz`、`sched100Hz` |
| `sdLogger` | `Components.SdLoggerSim`（相機板本機 SD） | 0x2080 | 100 Hz |
| `rg100Hz`／`rg1Hz` | `Svc.ActiveRateGroup` | 0x0200／0x0300 | — |
| CdhCore／ComCcsds／FileHandling | F´ 子拓樸（指令、事件、遙測、參數、GDS） | F´ 預設 | 1 Hz |

連線：`dsCam.imageOut`（2 KB 分塊，`Components.CamImageOut`）→ `sdLogger.dsImageIn`。

SIL 設定（`Top/DsCamTopology.cpp`）：`dsCam.configureSil(true, true)` → 開機自動上電一次、READY 後自動開始擷取一次
（SIL 沒有 DsMcu 送 CamLink 指令）。

## 跑 SIL
```bash
tools/verify/fprime_verify.sh --run 5 Components/Camera/CamTypes Components/Camera/DsCam \
  Components/Recorder/SdLogger Deployments/DsCam
```
預期看到：`Esp32CamSim OFF -> INIT -> SELFTEST -> READY`（`StateChanged`），遙測 `Status.frameCount` 每秒更新。
接 GDS：`fprime-gds` 開好後 `./Deployments_DsCam -a 127.0.0.1 -p 50000`，可以下 `CAPTURE_STOP`、`SIM_INJECT` 等指令。

## 已知限制（照實寫）
- `Esp32CamSim` 的模擬裝置只算幀數、**不產生像素**，所以 SIL 裡 `imageOut` 不會送資料，`sdLogger` 收不到影像；
  連線先接好，影像路徑等真機 `Esp32Cam`（或之後給 Sim 加假幀）再驗。
- SIL 沒有自動對 `sdLogger` 下 `START`（同 UsCam）。
- 地面預覽（`IGroundPreviewLink`，HTTP 144P）不在 F´ 拓樸內。

## 還沒定
- CamLink（DsMcu ↔ 相機板）指令鏈路、腳位：ESP32 平台 PR 的全箭腳位表。
