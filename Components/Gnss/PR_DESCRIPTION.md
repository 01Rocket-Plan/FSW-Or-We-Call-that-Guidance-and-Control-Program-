# [GNSS][梁承業] GnssDriver＋GnssDriverSim（TEL0157，GnssFix v0.2）— Edward PR #6 回覆

分支：`Gnss/gnss-driver`（本機提交；**請首席工程師推送**，執行者不推 GitHub）

- 安裝位置：**DS／US 皆可**（由拓撲決定；檔頭與本說明皆註明）。
- Base ID：依 `docs/conventions.md` 為 **GnssDriver `0x5000`／Sim `0x5080`**（組內曾暫定 `0x0A00` 已作廢）。

## Edward 審查回覆（commit on top of 90b1081）

### 1. FPP 埠註釋（指令分發器等）
- `cmdIn`／`cmdRegOut`／`cmdResponseOut` 加 Traditional Chinese `@`：連接**指令分發器**（Svc.CommandDispatcher／cmdDisp）。
- 其餘埠補齊短註釋：`schedIn`（rg1Hz）、`fixOut`（tlm／sdLogger only）、`uartRecv`／`uartRecvReturn`、`timeCaller`、`prmGetOut`／`prmSetOut`、`tlmOut`、`logOut`／`logTextOut`。Sim 同步。

### 2. Gate C6 — 飛行中失鎖／斷線（不只寫啟動條件）
來源：團隊 Drive「01RocketPlan專案須知」→ `gates-A-to-H.md` **C6**（及 E4 GNSS 列對齊 C6）。**閘門原文僅留本機 `.local_gate*`，不進 bundle／不外傳。**

Gate 定案（摘要）：
- 100 m 要 GNSS；**失鎖不准自己中止**；重試。
- `FIX_FAILED` **只報一次**；只影響**紀錄／落點顯示**；閉環靠 IMU，開傘不信 GNSS 高度。
- E4：失鎖依現況 sketch 重鎖；完全逾時後**報錯**（事件），不是把自己中止。

實作（`GnssCore`＋Driver／Sim）：
| 行為 | 實作 |
|------|------|
| 降級輸出 | READY 每 tick 仍出 `GnssFix`；`posValid=false`；`dataAge_s`↑ |
| 失鎖（仍收 NMEA） | `NO_FIX`／2D／HDOP 超標 → `FIX_LOST` edge 一次；維持 READY |
| UART 靜默 | `NODATA_WARN_TICKS`（預設 2）後 `NO_DATA` edge 一次；維持 READY |
| 逾時進 FAULT | `NODATA_FAULT_TICKS` 預設 **0＝永不**（C6）；>0 可選地面 BIT |
| 資料過舊 | `FIX_STALE_AGE_S`（預設 1.5 s）強制 `posValid=false` |
| 恢復 | 再得 3D+HDOP+未過舊 → `FIX_RECOVERED`；清除 latch |
| 遙測 | `DataAge`、`NoDataTicks`、`FixLost`（+原有 HsState／FixSeq／…） |
| 仍 FAULT | SELFTEST_FAIL、BUS_ERROR、STUCK（假設硬體異常） |

**假設（gate 未寫死，保守選定，見 `GnssCore.hpp`）：**
- A1. 接收機內部重鎖；驅動以 rg1Hz 輪詢（不在驅動內 sleep 100 ms）。
- A2. STUCK → FAULT。
- A3. BUS_ERROR／SELFTEST_FAIL → FAULT。
- A4. `FIX_STALE_AGE_S` 預設 1.5 s。
- A5. `NO_DATA`／`FIX_LOST` 各為一段 episode 的 edge；位元組／定位恢復後可再報。

### 3. 測試
- 主機 T1–T8 更新以符合 C6；新增 **T10 失鎖、T11 UART 靜默、T12 恢復、T13 拍動**。
- `fpp-check`、`fpp-to-cpp` + F´ v4.4.1 `-Wall`、`git bundle verify`。

## 內容（既有）
- `Components/Gnss/GnssDriver/`：真驅動（`GnssCore` NMEA GGA/RMC/VTG/GSA；握手 OFF→INIT→SELFTEST→READY/FAULT）。
- `Components/Gnss/GnssDriverSim/`：同一核心 + `GnssSimDevice`；`SIM_INJECT` 含 `NO_FIX`／`NO_DATA`。
- `Components/Gnss/fpp_stubs/`：GNSS 專用 stubs（**未改** EnvSensing stubs）。
- `RocketTypes`：僅 GNSS 標註區段；未改 `GnssFix` 欄位。

## 已知缺口
1. 主幹無 vendored `lib/fprime`，未做完整 F´ CMake 連結／實機 UART；編譯檢查對 `/workspace/fprime441`。
2. 拓撲尚未接 rg1Hz、prmDb、tlmSend、sdLogger、UART 驅動。
3. 僅 NMEA；UBX 未做。
4. 共用機以 `python3 -m ziglang c++` 編譯。
5. 執行者不推遠端；請首席工程師推 `Gnss/gnss-driver` 更新 PR #6。
