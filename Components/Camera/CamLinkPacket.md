# CamLink 封包格式提案（ESP32-CAM ↔ DS 飛控，I2C）

> 提案文件，**不**進入 `RocketTypes`。實體腳位／位址定案後再實作 I2C 驅動。
> 本鏈路**只**承載指令、狀態與幀數通知；影像像素禁止走此鏈路。

## 實體層

| 項目 | 值 |
|------|-----|
| 匯流排 | I2C |
| 時脈 | 預設 400 kHz（`I2C_CLOCK_HZ` 可調） |
| 腳位 | placeholder（對齊 Anatoly 腳位表） |
| 7-bit 位址 | TBD |

## 通用幀格式

| 欄位 | 長度 | 說明 |
|------|------|------|
| `SOF` | 1 | 固定 `0xA5` |
| `type` | 1 | 訊息類型（見下表） |
| `length` | 1 | `payload` 位元組數（0–64） |
| `payload` | `length` | 類型相關資料 |
| `crc8` | 1 | 對 `type‥payload` 的 CRC-8/ATM（poly 0x07，init 0x00） |

最大幀長 = 1 + 1 + 1 + 64 + 1 = 68 bytes（適合 I2C 單次交易）。

## 訊息類型

| `type` | 方向 | 名稱 | payload |
|--------|------|------|---------|
| `0xA1` | FC→CAM | `PING` | 無 |
| `0xA2` | CAM→FC | `PONG` | 無 |
| `0xD0` | CAM→FC | `INFO` | `model:u8`, `role:u8` |
| `0xB1` | FC→CAM | `CAPTURE_START` | 無 |
| `0xB2` | FC→CAM | `CAPTURE_STOP` | 無 |
| `0xB0` | FC→CAM | `POWER_ON` / 軟啟（可選，亦可用指令埠） | 無 |
| `0xB3` | FC→CAM | `RESET` | 無 |
| `0xC0` | CAM→FC | `FRAME_NOTIFY` | `index:u32le`, `bytes:u32le`, `width:u16le`, `height:u16le`（12 bytes，**無像素**） |
| `0xE0` | CAM→FC | `STATUS` | `camState:u8`, `fault:u8`, `frameCount:u32le`（6 bytes；對應 CamStatus.camState） |

## 地面預覽（不在本文件鏈路）

預覽走獨立 `IGroundPreviewLink`：HTTP 成幀、144P（256×144）、1～3 FPS。實體層（WiFi／RF）TBD，**不得**封進上述 I2C 幀。

## 完整錄影

留在 ESP32-CAM 本機 SD（`ILocalSdStore`），不經本鏈路、不進飛控遙測。

## 備註

- Host／Sim 目前使用精簡單位元組指令（與上表 `type` 值對齊），待 I2C 驅動接入時再包上 `SOF`／`length`／`crc8`。
- CRC 與最大長度可在實作審稿時調整；變更須同步更新本文件與 Sim。
