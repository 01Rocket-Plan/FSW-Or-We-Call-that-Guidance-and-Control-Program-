# [GNSS][梁承業] GnssDriver＋GnssDriverSim（TEL0157，GnssFix v0.2）— Edward PR #6 回覆

分支：`Gnss/gnss-driver`（本機提交；**請首席工程師推送**，執行者不推 GitHub）

- 安裝位置：**DS／US 皆可**（由拓撲決定；檔頭與本說明皆註明）。
- Base ID：依 `docs/conventions.md` 為 **GnssDriver `0x5000`／Sim `0x5080`**（組內曾暫定 `0x0A00` 已作廢）。

## Edward 審查回覆 + 交叉審查補洞

### 1. FPP 埠註釋（指令分發器等）
- `cmdIn`／`cmdRegOut`／`cmdResponseOut`：Traditional Chinese `@` → **指令分發器**（Svc.CommandDispatcher／cmdDisp）。
- 其餘埠補齊短註釋（schedIn、fixOut、uart*、time、prmDb、tlm、log）。Sim 同步。

### 2. Gate C6／E4 — 飛行中失鎖／斷線／完全逾時
（閘門原文僅本機 `.local_gate*`；此處只引 **C6**／**E4**。）

| 行為 | 實作 |
|------|------|
| 降級輸出 | READY 每 tick 仍出 `GnssFix`；`posValid=false`；`dataAge_s`↑ |
| 失鎖（仍收 NMEA） | `FIX_LOST` edge；維持 READY |
| UART 靜默 | `NODATA_WARN_TICKS`（預設 2）→ `NO_DATA` edge |
| **E4 完全逾時** | `NODATA_TIMEOUT_TICKS`（預設 **10**＝10 s@rg1Hz）→ **`NODATA_TIMEOUT` edge 一次／episode**；**維持 READY** |
| 進 FAULT（靜默） | `NODATA_FAULT_TICKS` 預設 **0**（C6）；>0 可選地面 BIT |
| 資料過舊 | `FIX_STALE_AGE_S`（預設 1.5 s） |
| **READY 後 STUCK** | 視同失鎖：維持 READY、`posValid=false`、`dataAge`↑、`STUCK` edge 一次；恢復 → `FIX_RECOVERED` |
| **SELFTEST 中 STUCK** | 仍進 **FAULT**（帶起／BIT） |
| 恢復 | 3D+HDOP+未過舊+非 STUCK → `FIX_RECOVERED` |
| 遙測 | `DataAge`、`NoDataTicks`、`FixLost`（+ HsState／FixSeq／…） |

**假設（gate 未寫死，見 `GnssCore.hpp` A1–A6）：**
- A1. 接收機內部重鎖；驅動 rg1Hz 輪詢。
- A2. READY 後 STUCK＝降級；SELFTEST 中 STUCK＝FAULT。
- A3. BUS_ERROR／SELFTEST_FAIL → FAULT。
- A4. `FIX_STALE_AGE_S` 或 STUCK 中 → `posValid=false`。
- A5. `NODATA_TIMEOUT_TICKS` 預設 10；WARN／TIMEOUT／FAULT 分層；FAULT 預設關。
- A6. SELFTEST 需 ≥2 tick 且最新 GGA 非重複才進 READY（以便帶起偵測 STUCK）。

### 3. 測試
- T1–T8（T8：READY-STUCK 不進 FAULT）；T10–T12；**T13** 含 NO_FIX **與 NO_DATA** 拍動；**T14** `NODATA_TIMEOUT`；**T15** STUCK 降級＋恢復（dataAge／seq）；**T16** SELFTEST STUCK→FAULT。
- `fpp-check`、`fpp-to-cpp` + F´ v4.4.1 `-Wall`、`git bundle verify`。

## 內容（既有）
- `Components/Gnss/GnssDriver/`：真驅動（`GnssCore` NMEA GGA/RMC/VTG/GSA）。
- `Components/Gnss/GnssDriverSim/`：同一核心 + `GnssSimDevice`。
- `Components/Gnss/fpp_stubs/`：GNSS 專用 stubs（**未改** EnvSensing stubs）。
- `RocketTypes`：僅 GNSS 標註區段；未改 `GnssFix` 欄位。

## 已知缺口
1. 主幹無 vendored `lib/fprime`；編譯檢查對 `/workspace/fprime441`。
2. 拓撲尚未接 rg1Hz、prmDb、tlmSend、sdLogger、UART。
3. 僅 NMEA；UBX 未做。
4. 共用機以 `python3 -m ziglang c++` 編譯。
5. 執行者不推遠端；請首席工程師推 `Gnss/gnss-driver` 更新 PR #6。
