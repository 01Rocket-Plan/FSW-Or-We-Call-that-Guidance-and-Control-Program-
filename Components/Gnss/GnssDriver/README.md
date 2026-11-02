# GnssDriver

- 範圍：TEL0157 `gnssDriver`（真驅動）＋ `gnssDriverSim`，`fixOut`（GnssFix v0.2）
- 安裝位置：DS／US 皆可（拓撲決定）
- Base ID：`0x5000`（docs/conventions.md）；Sim `0x5080`。先前暫定 `0x0A00` 已作廢。
- 核心 `GnssCore` 與 F´ 無關，可主機單測；`schedIn`＝**rg100Hz**（10 ms）。**整數 tick 分頻**：`pollPeriodTicks = POLL_*_MS / rgPeriod_ms` → 預設 10／100／200；每 poll 一次 UART；不 sleep。

## Edward 2026-11-02 失鎖表（C6／E4）

| Stage | Poll | 自 last-good 邊界 |
|---|---|---|
| TRACKING | 100 ms | continuous |
| REACQ_FAST（失鎖） | 100 ms | 0 → **15 s** |
| REACQ_MED | 1000 ms | 15 → **25 s** |
| REACQ_SLOW | 2000 ms | 25 → **55 s** |
| FIX_FAILED | （續用 SLOW） | ≥ 55 s |

- **55 s 時鐘從最後一次合格 GGA 起算**（中斷真正開始的時刻）。`LOSS_NO_GGA_S` 偵測耗時算進 REACQ_FAST。
- 任一段成功 → **TRACKING**。進 `FIX_FAILED` 發一次：「請檢察環境開放條件以利天線運作」。
- 開機 `ACQ` ≤ **300 s**；逾時 → 一次 `INIT_ACQ_TIMEOUT` 後進 **FIX_FAILED**（維持 READY、SLOW 輪詢；可恢復）。**不**因 ACQ 逾時進 FAULT（A1）。`BUS_ERROR`／`SELFTEST_FAIL` 仍 → FAULT。
- **輸出／seq**：每次發出的 fix 都 `++seq`（有輸出才遞增）。新 GGA → 立刻發一筆；無新 GGA 的重鎖／靜默 → 最多 **1 Hz** 降級（`posValid=false`、`dataAge`↑）。
- **讀取緩衝 2048 B**：9600 8N1 → 960 B/s；2 s 含 GSV ≈ 1920 B，2048 留餘量。仍每 poll 一次 bus read。
- **C6**：不自己中止；`fixOut` 只接 tlm／sdLogger。
- **prmDb**：`HDOP_MAX`、`POLL_*_MS`、`REACQ_*_S`、`INIT_ACQ_TIMEOUT_S`、`SELFTEST_TIMEOUT_S`、`LOSS_NO_GGA_S`、`STUCK_POLLS`。
- 遙測含 **`LockStage`**。

假設見 `GnssCore.hpp`（A1–A6）。閘門只引 **C6**／**E4** ID；原文不進 repo。
