# DsCam（ESP32-CAM 模組，內建感測器 OV2640）

- 負責：相機組（Arthur）／子Bot1
- FPP 宣告：型號 `ESP32_CAM`、角色 `DS_CAM`
- 共用型別：`Components/Camera/CamTypes`（FPP 自動碼；2026-11-10 起不准手寫同名 C++ 型別，issue #36）
- 範圍：`Esp32CamCore` + `Esp32CamSimDevice` + F´ 外殼 `Esp32Cam`／`Esp32CamSim`
- 飛控連線：I2C → `ICamLink`；時脈預設 400 kHz；`I2C_ADDR`／`I2C_PIN_SDA`／`I2C_PIN_SCL` 為 placeholder（Anatoly 腳位表）
- 完整錄影：ESP32-CAM 本機 SD（`ILocalSdStore`）；預覽：`IGroundPreviewLink`（不經 ICamLink／遙測）
- 影像埠：`Components.CamImageOut`（與上級相機共用，issue #32）：2KB `Fw.Buffer` 分塊，`frameNumber`/`chunkIndex`/`CamChunkFlag`；`FRAME_MAX_BYTES` 截斷不計 `frameCount`、維持 READY
- Rate groups：1 Hz／100 Hz；不碰 1 kHz；不 sleep、不讀系統時間做逾時、不動態配置

封包提案見 `../CamLinkPacket.md`。
- **驗證**：`tools/verify/fprime_verify.sh --ut Components/Camera/CamTypes Components/Camera/DsCam`（host 測試已搬進 F´ UT／gtest）

## 時序模型與預設參數（`CamTiming.hpp`；SIL 測試發現，見 SSS_Mission_Status #5）

相機型號（首席 2026-11-18 確認）：**DsCam（下級）＝ ESP32-CAM 模組，內建感測器 OV2640**，下表的感測器數字都是 OV2640 的。
上級（主）相機 UsCam 是 **OV7670**（`../Ov7670Driver`，沒有硬體 JPEG）；**OV3640** 只是停在 fork 分支的備援、不在 main。三者規格不要混用。

**解析度（Edward 2026-11-24 決議）：DsCam 完整錄影維持 640×480 JPEG。** 別和下面兩個搞混：
- 「144p 灰階 1 fps」＝上級 UsCam（OV7670，PR #55；`../Ov7670Driver/Ov7670Driver.fpp`），已實作。
- 「144P（256×144）1～3 FPS」＝ DsCam 的**地面預覽** `IGroundPreviewLink`（`../CamLinkPacket.md`「地面預覽」），不是錄影；預覽還沒接線。

> ⚠ **下表 I2C／SCCB 的時間都是規格書計算值，未實測**；實測待 HIL（相機板上機量 SCL 波形與交易間隔）。
> OV2640 的 SCCB 數字只適用 ESP32-CAM 模組內部（ESP32↔OV2640）；上級 OV7670 的 SCCB 時序未建模、不給數字（待 HIL）。

數字都有出處（細節見 `CamTiming.hpp` 檔頭）；**相機組可再改**。

| 項目 | 值 | 出處 |
|---|---|---|
| I2C 每位元組 | 9 個 SCL 週期（8 資料＋ACK），交易開頭 1 個位址位元組 | NXP UM10204 Rev. 7.0 §3.1.6、§3.1.10 |
| ICamLink I2C 時脈 | 400 kHz（repo 既有預設） | `Esp32CamCore::kDefaultI2cClockHz`、UM10204 Fast-mode |
| 幀通知 13 B 的 I2C 時間 | (1+13)×9 / 400 kHz = 315 µs | 由上兩列算出 |
| CamLink 最大幀 68 B 的 I2C 時間 | (1+68)×9 / 400 kHz = 1552.5 µs | `../CamLinkPacket.md` 最大幀長＋上兩列 |
| SCCB 寫一個暫存器（ESP32↔OV2640） | 3 phase × 9 = 27 個週期；100 kHz 下 270 µs | OmniVision SCCB Functional Spec（3-phase write） |
| SCCB 時脈（ESP32↔OV2640） | 預設 100 kHz（上限 400 kHz） | esp32-camera Kconfig `SCCB_CLK_FREQ`；OV2640 DS v1.6 Table 9 |
| XCLK | 20 MHz | esp32-camera README 範例 `xclk_freq_hz` |
| `JPEG_QUALITY` | 12（0–63，越小畫質越好） | esp32-camera README 範例；`esp_camera.h` |
| 640×480 感測器最大幀率 | 30 fps（VGA 走 SVGA 模式） | OV2640 DS v1.6 p.1；esp32-camera `sensors/ov2640.c` `set_framesize()` |
| Sim 幀間隔 | 5 tick × 10 ms = 50 ms（20 fps ≤ 30 fps，有 `static_assert`） | `Esp32CamSimDevice::kSimFrameIntervalTicks` |

時間公式只算 SCL 週期（下限），不含 START／STOP 建立保持時間與 clock stretching；規格書計算值，未實測，實測待 HIL。

**沒有出處、不給數字（待冷測量）**：JPEG 一幀位元組數（看畫面與品質；sim 仍用 1024 B 占位）、20 MHz XCLK 下的實際幀率。
`JPEG_QUALITY` 是 esp32-camera 的 0–63 刻度，**不能**和 libjpeg／PIL 的 0–100 品質直接換算。
