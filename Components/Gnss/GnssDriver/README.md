# GnssDriver

- 範圍：TEL0157 `gnssDriver`（真驅動）＋ `gnssDriverSim`，`fixOut`（GnssFix v0.2）
- 安裝位置：DS／US 皆可（拓撲決定）
- Base ID：`0x5000`（docs/conventions.md）；Sim `0x5080`。先前暫定 `0x0A00` 已作廢。
- 核心 `GnssCore` 與 F´ 無關，可主機單測；`schedIn`＝**rg100Hz**，依 `LockStage` 以 tick divider 決定 poll（含一次 UART `readBytes`）；不 sleep。

## Edward 2026-11-02 失鎖表（C6／E4）

| Stage | Poll | Duration |
|---|---|---|
| TRACKING | 100 ms | continuous |
| REACQ_FAST（失鎖） | 100 ms | 15 s |
| REACQ_MED | 1000 ms | 10 s |
| REACQ_SLOW | 2000 ms | 30 s |
| FIX_FAILED | （續用 SLOW） | — |

- 三段重鎖合計 **55 s**；任一段成功 → **TRACKING**。
- 進 `FIX_FAILED` 發一次：「請檢察環境開放條件以利天線運作」（原文）。
- 開機 `ACQ` ≤ **300 s**；逾時 → `FAULT`＋`INIT_ACQ_TIMEOUT`（假設 A1）。
- **C6**：不自己中止；`fixOut` 只接 tlm／sdLogger。重鎖中 `posValid=false`、`dataAge`↑。
- 失鎖條件：劣質 GGA、或 `LOSS_NO_GGA_S` 內無新 GGA（**含 UART 靜默**）、或 `STUCK_POLLS`。舊 `NODATA_*`／`FIX_STALE_AGE_S` 已併入階段機後刪除。
- **prmDb**：`HDOP_MAX`、`POLL_*_MS`、`REACQ_*_S`、`INIT_ACQ_TIMEOUT_S`、`SELFTEST_TIMEOUT_S`、`LOSS_NO_GGA_S`、`STUCK_POLLS`。
- 遙測含 **`LockStage`**。

假設見 `GnssCore.hpp`（A1–A6）。閘門只引 **C6**／**E4** ID；原文不進 repo。
