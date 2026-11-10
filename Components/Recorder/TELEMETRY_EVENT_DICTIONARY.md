# 01RocketPlan 整車遙測／事件紀錄字典（合約 §9）

- 作者：紀錄組子Bot1
- 對照 main：`34a348e`；SdLogger FPP／實作基底：子Bot2 `7458965`（image chunk；含 #21 `3346740` Core+Sim）
- 依據：介面合約 v0.1 §9 TELEMETRY / EVENTS / LOGGING；`docs/conventions.md` §5 Base ID；main 已合併型別；Bot2 `SdLogger.fpp` 為 ID 準據
- 用途：sdLogger（子Bot2）依本表決定「記什麼」；各組仍擁有自己元件的 FPP 宣告與絕對 ID
- 規則：只有紀錄組（`0x2000–0x2FFF`）在本交付寫死具體 ID；其他組標「待該組定」或「待該組補」；**勿改** `SdLogger.fpp`（子Bot2 擁有）

## LOGGING 規則（全頻道）

| 規則 | 說明 |
|------|------|
| seq 連續 | 每個紀錄頻道／樣本序號連續；缺口發 `SeqGap(expected,got)`（0x2004）；影像 frame／chunk 缺口見 `ImageFrameGap`／`ImageChunkGap` |
| 時間戳單調 | `t_s` 只取 F´ 時間服務，不得 sleep／系統時鐘；倒退發 `TimeNonMonotonic(prev_s,now_s)`（0x2005） |
| 佇列滿 | 丟最新並發事件，**不得阻塞**呼叫端（紀錄佇列→`QueueFullDrop`；影像分塊佇列→`ImageChunkDrop`） |

---

## A. §9 TELEMETRY → 擁有元件與頻道名

| §9 項目 | 擁有元件 | 建議／既有頻道或欄位名 | 組別 Base | 具體 ID |
|---------|----------|------------------------|-----------|---------|
| flightMode | `FlightMode` | `mode`（型別 `RocketTypes.FlightModeId`／`ModePort`） | MCU `0x1400` | 待該組補（殼僅介面） |
| quaternion attitude | `Estimator` | `StateOut.q_wxyz`（經 `StateData`） | MCU `0x1100` | 待該組補 |
| tilt from vertical | `Estimator` | `StateOut.tilt_rad` | MCU `0x1100` | 待該組補 |
| velocity estimate | `Estimator` | `StateOut.vel_enu_mps` + `velValid` | MCU `0x1100` | 待該組補 |
| control-surface command angle | `Controller` → `ActuatorMgr` | `FinCmd.fin_rad[4]`（rad） | MCU `0x1200`／`0x1300` | 待該組補 |
| control-surface command µs | `ActuatorMgr` → `ServoDriver` | `ServoCmd.pulse_us[4]`；遙測 `lastPulse0_us`…`lastPulse3_us` | 伺服 `0x3000` | 待該組定（ServoDriver 已有 tlm，opcode 待拓樸） |
| 各元件 handshake 狀態 | 各驅動 | `drvState`／`HsState`：`RocketTypes.DrvState` | 見各列 | 見下表 |
| voltage rails | `PowerMonitor` | `PowerRailTlm`（`BATT_MAIN`／`SERVO_5V`／`LOGIC_3V3`；`voltage_V`／`current_A`／`sample_age_s`） | 電源 `0x6000` | 待該組定 |
| GNSS fix | `GnssDriver` | 埠 `fixOut`→`RocketTypes.GnssFix`；tlm `FixSeq` | GNSS `0x5000` | 待該組定 |
| GNSS data age | `GnssDriver` | tlm `DataAge`（秒；欄位名無 `_s` 後綴，見 mismatch） | GNSS `0x5000` | 待該組定 |
| 每佇列／緩衝池 HWM | 見 §C | `*Hwm` | 各組 | 紀錄組已定 SdLogger（含 `ImageQueueHwm`）；其餘待該組補 |
| camera image chunk 計數／丟棄／位元組 | `SdLogger`（經 `usImageIn`／`dsImageIn`） | `ImageChunkCount`／`ImageDropCount`／`ImageBytesWritten`／`ImageFrameCount` | 紀錄 `0x2000` | **0x200D**–**0x2010**（子Bot2） |
| camera image 連續性監視 | `SdLogger` | `LastImageFrame`／`LastImageChunk` | 紀錄 `0x2000` | **0x2012**／**0x2013**（子Bot2） |

### Handshake（§9「each component's handshake status」）

| 元件 | 既有遙測名 | Base | 具體 ID |
|------|------------|------|---------|
| `ImuDriver` | `drvState` | `0x4000` | 待該組定 |
| `BaroDriver` | `HsState` | `0x4100` | 待該組定 |
| `UsMpu6050` | `drvState` | `0x4200` | 待該組定 |
| `UsDht22` | `HsState` | `0x4300` | 待該組定 |
| `GnssDriver` | `HsState` | `0x5000` | 待該組定 |
| `ServoDriver`（TAIL／GIMBAL） | `drvState` + `model`/`role`（`ServoTypes`） | `0x3000`／`0x3200` | 待該組定 |
| `SepServoDriver` | `drvState` + `model`/`role` | `0x3100` | 待該組定 |
| `SdLogger` | `HsState`（`RocketTypes.DrvState`；另有 `SdStatus`／`EnterReady`／`EnterFault`） | `0x2000` | **0x2008**（子Bot2） |
| `PowerMonitor`／`UvStateComp` | 無 `DrvState` handshake tlm | `0x6000`／`0x6200` | 待該組補（§9 要求 handshake；main 未宣告） |
| `FlightMode`／`Estimator`／`Controller`／`ActuatorMgr`／`Platform` | 骨架 | MCU 段 | 待該組補 |
| `TlmSend`／`Framer`／`ComQueue`／`UplinkAuth` | 骨架 | 無線 `0x8000+` | 待該組補 |
| `DsCam`／`UsCam` | 骨架 | 相機 `0x7000+` | 待該組補 |

---

## B. §9 EVENTS → 擁有元件與事件名

| §9 項目 | 擁有元件 | 建議／既有事件名 | Base | 具體 ID |
|---------|----------|------------------|------|---------|
| every mode transition（含 trigger：sensed vs backup timer） | `FlightMode` | 建議 `ModeTransition(from,to,cause)`；cause 區分感測／備援計時 | `0x1400` | 待該組補（Bot 不得寫轉移邏輯） |
| rate group slip | `Platform` | 建議 `RateGroupSlip(group,late_s)` | `0x1000` | 待該組補 |
| gyro saturation | `ImuDriver`／`UsMpu6050` | 既有 `GyroSaturated` | `0x4000`／`0x4200` | 待該組定 |
| control-surface limit／saturation | `ServoDriver`／`ActuatorMgr` | 既有 `PulseClamped`；限幅／飽和應用層事件 | `0x3000`／`0x1300` | 伺服已有 clamp；應用飽和 待該組補 |
| IMU fail and recovery | `ImuDriver` | 既有 `EnterFault`／`EnterReady`／`Failover` | `0x4000` | 待該組定 |
| bus error and recovery | `ImuDriver`／`UsMpu6050`／`BaroDriver`／`GnssDriver` | 既有 `BusError`／`Fault`；「recovery」多以 `EnterReady`／`StateChanged` 表達 | 環境／GNSS | 待該組定 |
| command rejected | `UplinkAuth`／`FlightMode` | 建議 `CmdRejected(cmd,reason)`（白名單／密碼錯） | `0x8300`／`0x1400` | 待該組補 |
| each SEP commanded separation | `SepServoDriver` | 既有 `SepFire(ch)` | `0x3100` | 待該組定 |
| parachute deploy commanded | `FlightMode` | 建議 `ChuteDeployCmd(stage,cause)` | `0x1400` | 待該組補 |
| undervoltage load shed | `UvStateComp` | 既有 `shedL1Requested`／`shedL2Requested`／`levelChanged` | `0x6200` | 待該組定 |
| queue-full drop | 各佇列擁有者 | `SdLogger.QueueFullDrop`；`ComQueue` 等 | 見 §C／§D | SdLogger 已定；其餘待該組補 |
| channel seq discontinuity（LOGGING） | `SdLogger` | `SeqGap(expected,got)` | `0x2000` | **0x2004** |
| timestamp non-monotonic（LOGGING） | `SdLogger` | `TimeNonMonotonic(prev_s,now_s)` | `0x2000` | **0x2005** |
| image queue-full drop（LOGGING／相機分塊） | `SdLogger` | `ImageChunkDrop(dropped,queueDepth)` | `0x2000` | **0x2006** |
| image frameNumber discontinuity | `SdLogger` | `ImageFrameGap(expected,got)` | `0x2000` | **0x2007** |
| image chunkIndex discontinuity | `SdLogger` | `ImageChunkGap(frameNumber,expected,got)` | `0x2000` | **0x2008** |
| image TRUNCATED flag seen | `SdLogger` | `ImageTruncatedSeen(frameNumber,chunkIndex)` | `0x2000` | **0x2009** |

---

## C. 佇列與緩衝池 HWM（§9「every queue and buffer pool」）

| 佇列／池 | 擁有元件 | HWM 頻道名 | Base | 具體 ID |
|----------|----------|------------|------|---------|
| SdLogger 紀錄佇列 | `SdLogger` | `RecordQueueHwm` | `0x2000` | **0x2002**（local 0x02） |
| SdLogger SD 寫入緩衝池 | `SdLogger` | `SdWritePoolHwm` | `0x2000` | **0x2003**（local 0x03） |
| SdLogger 影像分塊佇列 | `SdLogger` | `ImageQueueHwm` | `0x2000` | **0x2011**（local 0x11） |
| 通訊佇列 | `ComQueue` | 待命名 | `0x8200` | 待該組補（骨架） |
| F´ `bufferManager` 等標準池 | `Platform`／F´ 標準 | 待命名 | `0x0000–0x0FFF` | 待該組補 |
| 其他組內部佇列 | 各組 | 待命名 | 各組段 | 待該組補 |

---

## D. Recorder 擁有 — SdLogger ID 表（絕對 ID = base `0x2000` + local）

### Telemetry

| Local | Absolute | Name | Type | SI／語意 |
|------:|----------|------|------|----------|
| 0x00 | **0x2000** | `WriteCount` | U32 | 成功寫入筆數 |
| 0x01 | **0x2001** | `DropCount` | U32 | 丟棄筆數 |
| 0x02 | **0x2002** | `RecordQueueHwm` | U32 | 紀錄佇列高水位（depth） |
| 0x03 | **0x2003** | `SdWritePoolHwm` | U32 | SD 寫入緩衝池高水位 |
| 0x04 | **0x2004** | `SdErrorCount` | U32 | SD／媒體錯誤累計 |
| 0x05 | **0x2005** | `SdStatus` | `SdCardStatus` | OK／INIT／BUSY／CARD_MISSING／IO_ERROR／FULL |
| 0x06 | **0x2006** | `LastSeq` | U32 | 最近接受序號 |
| 0x07 | **0x2007** | `LastT_s` | F64 | 最近接受時間戳（s） |
| 0x08 | **0x2008** | `HsState` | `RocketTypes.DrvState` | Logger handshake（子Bot2） |
| 0x09 | **0x2009** | `LoggerSeq` | U32 | 最近 logger 連續寫入序號（子Bot2） |
| 0x0A | **0x200A** | `ProducerSeq` | U32 | 最近寫入之 SampleHeader.seq（子Bot2） |
| 0x0B | **0x200B** | `CardModel` | `SdCardModel` | Micro SDHC UHS-I 16GB（子Bot2） |
| 0x0C | **0x200C** | `CardRole` | `SdCardRole` | DS_LOG / US_LOG（子Bot2） |
| 0x0D | **0x200D** | `ImageChunkCount` | U32 | 成功寫入影像分塊數（子Bot2） |
| 0x0E | **0x200E** | `ImageDropCount` | U32 | 影像佇列滿丟棄數（子Bot2） |
| 0x0F | **0x200F** | `ImageFrameCount` | U32 | 完成影像幀數（見 END）（子Bot2） |
| 0x10 | **0x2010** | `ImageBytesWritten` | U32 | 累計影像 payload bytes（子Bot2） |
| 0x11 | **0x2011** | `ImageQueueHwm` | U32 | 影像分塊佇列高水位（子Bot2） |
| 0x12 | **0x2012** | `LastImageFrame` | U32 | 最近接受 frameNumber（子Bot2） |
| 0x13 | **0x2013** | `LastImageChunk` | U32 | 最近接受 chunkIndex（子Bot2） |

### Events

| Local | Absolute | Name | Severity | 欄位 |
|------:|----------|------|----------|------|
| 0x00 | **0x2000** | `QueueFullDrop` | warning high | `dropped: U32`, `queueDepth: U32` |
| 0x01 | **0x2001** | `SdError` | warning high | `status: SdCardStatus`, `code: U8` |
| 0x02 | **0x2002** | `EnterReady` | activity high | — |
| 0x03 | **0x2003** | `EnterFault` | warning high | `reason: U8` |
| 0x04 | **0x2004** | `SeqGap` | warning high | `expected: U32`, `got: U32`（upstream seq discontinuity） |
| 0x05 | **0x2005** | `TimeNonMonotonic` | warning high | `prev_s: F64`, `now_s: F64`（timestamp went backwards） |
| 0x06 | **0x2006** | `ImageChunkDrop` | warning high | `dropped: U32`, `queueDepth: U32`（影像佇列滿丟最新） |
| 0x07 | **0x2007** | `ImageFrameGap` | warning high | `expected: U32`, `got: U32`（frameNumber 不連續） |
| 0x08 | **0x2008** | `ImageChunkGap` | warning high | `frameNumber: U32`, `expected: U32`, `got: U32` |
| 0x09 | **0x2009** | `ImageTruncatedSeen` | warning high | `frameNumber: U32`, `chunkIndex: U32`（CamChunkFlag.TRUNCATED） |

影像輸入：`usImageIn: CamImageOut`、`dsImageIn: CamImageOut`（相機組 `CamTypes` 定義，兩顆相機共用）（2KB 分塊；flags END=1／TRUNCATED=2）；獨立 `IMAGE_QUEUE_DEPTH`（預設 8）；頻道 `RecordChannel::IMAGE=11`。

Sim 實例：base **`0x2080`**，local 對照相同（abs = 0x2080 + local）。

FPP 硬體型號／角色：`SdCardModel.MICRO_SDHC_UHS_I_16GB`（媒體卡）；`SdCardRole` DS_LOG／US_LOG。讀卡模組／轉接板型號：**TBD**。SD 狀態用 `SdCardStatus`。

---

## E. §9 與 main 不一致／缺漏（不臆造）

1. **`GnssDriver` tlm `DataAge`**：語意為秒，名稱無 `_s` 後綴（慣例要求 `_s`）。字典沿用既有名，標 mismatch。
2. **Handshake**：`PowerMonitor`／`UvStateComp`／MCU 應用殼／無線／相機在 main **無** `DrvState` handshake 遙測；§9 要求 each component — 標「待該組補」。
3. **Mode transition + cause**、**parachute deploy**、**rate group slip**、**command rejected**：main 僅骨架或禁止 Bot 實作 — 「待該組補」。
4. **ComQueue／bufferManager HWM**：元件骨架，無頻道名 — 「待該組補」。
5. **控制面角度 vs µs**：main 已拆成 `FinCmd.fin_rad` 與 `ServoCmd.pulse_us`（符合「角度與 µs」兩者都要記）；應用層 tlm 尚缺（Estimator／Controller／ActuatorMgr 骨架）。
6. **`EnvSample.humidity_pct`／`HumTempSample`**：§9 TELEMETRY 清單未點名，但 main 註明只送 Recorder — 建議 sdLogger 仍記錄樣本埠，不另占 §9 強制 tlm ID。
7. **事件與遙測共用 local 編號空間**：F´ 實例化時 tlm／event opcode 空間分開；表中 Absolute 以「base+local」文件化，拓樸 `baseId` 指派後以 Ac 產物為準。

---

## F. 計數摘要

| 類別 | 數量 |
|------|------|
| §9 TELEMETRY 映射列（主表） | 14 |
| Handshake 列 | 13 |
| §9 EVENTS 映射列 | 17 |
| HWM 列（含占位） | 6 |
| **SdLogger 具體遙測 ID** | **20**（0x2000–0x2013；含 Bot2 0x2008–0x200C 與影像 0x200D–0x2013） |
| **SdLogger 具體事件 ID** | **10**（local 0x00–0x09；abs 0x2000–0x2009；含影像 0x2006–0x2009） |
