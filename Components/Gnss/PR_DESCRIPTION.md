# [GNSS][梁承業] GnssDriver＋GnssDriverSim — Edward 2026-11-02 失鎖表

分支：`Gnss/gnss-driver`（本機提交；**請首席工程師推送**，執行者不推 GitHub）

- 安裝位置：**DS／US 皆可**。Base ID：**GnssDriver `0x5000`／Sim `0x5080`**。
- 規格來源：**Edward 2026-11-02 失鎖表**。閘門只引 **C6**／**E4** ID。

## 失鎖／重鎖階段機

| Stage | Poll | 自 last-good 邊界 |
|---|---|---|
| TRACKING | 100 ms | continuous |
| REACQ_FAST | 100 ms | → 15 s |
| REACQ_MED | 1000 ms | → 25 s |
| REACQ_SLOW | 2000 ms | → 55 s |
| FIX_FAILED | SLOW | ≥ 55 s |

- **55 s 從最後合格 GGA 起算**；`LOSS_NO_GGA_S` 偵測時間計入 FAST。
- 任一段成功 → TRACKING。`FIX_FAILED` → 原文「請檢察環境開放條件以利天線運作」一次。
- 開機 ACQ ≤ 300 s → 一次 `INIT_ACQ_TIMEOUT` 後進 FIX_FAILED（不 FAULT；可恢復）。
- `schedIn`＝rg100Hz（10 ms）；**整數 tick 分頻** `POLL_*_MS/10` → 10／100／200；每 poll 一次 UART；讀取緩衝 **2048 B**。
- **seq**：每次發出的 fix 都 `++seq`。新 GGA → 立刻發一筆；無新 GGA 時最多 1 Hz 降級（`posValid=false`、`dataAge`↑）。

### prmDb
`HDOP_MAX`、`POLL_*_MS`、`REACQ_*_S`、`INIT_ACQ_TIMEOUT_S`、`SELFTEST_TIMEOUT_S`、`LOSS_NO_GGA_S`、`STUCK_POLLS`。

**假設（A1–A6）：**
- A1. ACQ 逾時 → `INIT_ACQ_TIMEOUT` 一次＋`FIX_FAILED`（READY／SLOW，可回 TRACKING）。`BUS_ERROR`／`SELFTEST_FAIL` → FAULT。
- A2. `FIX_FAILED` 後 SLOW 輪詢，鎖定成功回 TRACKING。
- A3. 失鎖＝劣質 GGA／`LOSS_NO_GGA_S`／`STUCK_POLLS`。
- A4. 合格 3D＝`FIX_3D`∧`hdop<=HDOP_MAX`；重鎖中 `posValid=false`。
- A5. 空 poll 正常；不以單次空讀當失鎖。
- A6. `TRUNCATED`／`BAD_CHECKSUM` 只計數。

## 測試
Host T9（Sim）＋ **T1–T12**（含 12／20／60 s 階段進入時刻與 poll 數、emit／dataAge、ACQ→FIX_FAILED）。`fpp-check`、`fpp-to-cpp` + `-Wall`、`git bundle verify`。

## 已知缺口
拓撲尚未接 rg100Hz／prmDb／tlm／sdLogger／UART；僅 NMEA；執行者不推遠端。
