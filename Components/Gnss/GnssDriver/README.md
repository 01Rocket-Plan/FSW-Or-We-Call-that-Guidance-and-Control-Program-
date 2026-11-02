# GnssDriver

- 範圍：TEL0157 `gnssDriver`（真驅動）＋ `gnssDriverSim`，`fixOut`（GnssFix v0.2）
- 安裝位置：DS／US 皆可（拓撲決定）
- Base ID：`0x5000`（docs/conventions.md）；Sim `0x5080`。先前暫定 `0x0A00` 已作廢。
- 核心 `GnssCore` 與 F´ 無關，可主機單測；UART 適配為 ByteStream 推入 + 每 tick 一次 `readBytes`。

## Gate C6（飛行中失鎖／斷線）

- **不自己中止**：READY 後 UART 靜默或 NMEA 失鎖（NO_FIX／2D／HDOP 超標／資料過舊）維持 `READY`，持續輸出。
- **降級**：`posValid=false`，`dataAge_s` 隨最後一筆新 GGA 增長；`FIX_LOST`／`NO_DATA` 各以 edge 報一次，恢復報 `FIX_RECOVERED`。
- **prmDb**：`HDOP_MAX`、`SELFTEST_TICKS`、`NODATA_WARN_TICKS`、`NODATA_FAULT_TICKS`（預設 **0＝永不因靜默進 FAULT**）、`STUCK_TICKS`、`FIX_STALE_AGE_S`。
- **仍進 FAULT**：`SELFTEST_FAIL`、`BUS_ERROR`、`STUCK`（假設：硬體卡死）。`NODATA_FAULT_TICKS>0` 可選地面 BIT。
- **閉環**：`fixOut` 只接 tlmSend／sdLogger；控制不依賴 GNSS（C6）。

假設見 `GnssCore.hpp` 檔頭（A1–A5）。
