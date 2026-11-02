# [GNSS][梁承業] GnssDriver＋GnssDriverSim — Edward 2026-11-02 失鎖表

分支：`Gnss/gnss-driver`（本機提交；**請首席工程師推送**，執行者不推 GitHub）

- 安裝位置：**DS／US 皆可**。Base ID：**GnssDriver `0x5000`／Sim `0x5080`**。
- 規格來源：**Edward 2026-11-02 失鎖表**（取代先前「維持 READY、只發事件」對 C6／E4 的解讀）。閘門只引 **C6**／**E4** ID。

## 失鎖／重鎖階段機

| Stage | Poll interval | Duration |
|---|---|---|
| TRACKING | 100 ms | continuous |
| REACQ_FAST（on loss） | 100 ms | 15 s |
| REACQ_MED | 1000 ms | 10 s |
| REACQ_SLOW | 2000 ms | 30 s |
| FIX_FAILED | （續 SLOW） | — |

- 三段合計 **55 s**。任一段重鎖成功 → **TRACKING**。
- 進 `FIX_FAILED` → 一次 `FixFailedOpenSky`：「請檢察環境開放條件以利天線運作」。
- 開機 `ACQ` ≤ **300 s**；逾時 → `FAULT`＋`INIT_ACQ_TIMEOUT`。
- `schedIn`＝**rg100Hz**；tick divider 決定 100／1000／2000 ms poll；每 poll 一次 UART；不 sleep。
- 重鎖中 `posValid=false`、`dataAge` 增長；C6 不中止；`fixOut` → tlm／sdLogger only。
- UART 靜默與 STUCK 併入失鎖條件；刪除冗餘 `NODATA_*`／`FIX_STALE_*` 參數。

### prmDb
`HDOP_MAX`、`POLL_TRACKING_MS`、`POLL_REACQ_FAST_MS`、`POLL_REACQ_MED_MS`、`POLL_REACQ_SLOW_MS`、`REACQ_FAST_S`、`REACQ_MED_S`、`REACQ_SLOW_S`、`INIT_ACQ_TIMEOUT_S`、`SELFTEST_TIMEOUT_S`、`LOSS_NO_GGA_S`、`STUCK_POLLS`。

### 遙測
`HsState`、`LockStage`、`FaultCount`、`BadChecksumCount`、`TruncatedCount`、`FixSeq`、`DataAge`。

**假設（`GnssCore.hpp` A1–A6）：**
- A1. 開機 ACQ 逾時 → `FAULT`（鎖存至 `start()`）；屬 BIT，非飛行重鎖。
- A2. `FIX_FAILED` 後仍以 SLOW 輪詢，鎖定成功回 TRACKING（C6 不中止）。
- A3. 失鎖＝劣質 GGA ／ `LOSS_NO_GGA_S` 無新 GGA（含靜默）／ `STUCK_POLLS`。
- A4. 合格 3D＝`FIX_3D` 且 `hdop<=HDOP_MAX`；重鎖中 `posValid=false`。
- A5. NMEA 常 1 Hz；空 poll 正常，不以單次空讀當失鎖。
- A6. `BUS_ERROR` → `FAULT`；`TRUNCATED`／`BAD_CHECKSUM` 只計數。

## 測試
- Host T9（SimDevice）＋ T1–T10（LockStage）：各階段轉換與計時、各階段恢復、`FIX_FAILED` 只發一次、5 分鐘級 ACQ 逾時（縮時）、拍動、靜默／STUCK。
- `fpp-check`、`fpp-to-cpp` + F´ v4.4.1 `-Wall`、`git bundle verify`。

## 內容
- `Components/Gnss/GnssDriver/`、`GnssDriverSim/`；`fpp_stubs/`（未改 EnvSensing stubs）。
- `RocketTypes`：僅 GNSS 標註；未改 `GnssFix` 欄位。

## 已知缺口
1. 主幹無 vendored `lib/fprime`；編譯檢查對 `/workspace/fprime441`。
2. 拓撲尚未接 **rg100Hz**、prmDb、tlmSend、sdLogger、UART。
3. 僅 NMEA；UBX 未做。
4. 共用機以 `python3 -m ziglang c++` 編譯。
5. 執行者不推遠端；請首席工程師推 `Gnss/gnss-driver`。
