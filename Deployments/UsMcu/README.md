# UsMcuDeployment

US 飛控 MCU（獨立板）。**不含控制律**：不安裝 Controller、ActuatorMgr、ServoDriver（尾翼），也不負責級間分離（分離時機判斷、SEP_FAIL 與分離供電在 DS，見 gate A9）。

**本版是 Linux SIL**（F´ v4.4.1）。建置與執行：`tools/sil/build_mcu_sil.sh --run 3`（見 `Deployments/DsMcu/README.md`）。

## 速率群組（基準時鐘 1 ms）
| 群組 | 成員 |
|---|---|
| rg1kHz | `flightMode`（role US）→ `platform`（最後，踢狗） |
| rg100Hz | `usMpu6050` → `sdLogger`（flush；寫入在 schedIn，輸入埠只進佇列、不擋 100 Hz） |
| rg1Hz | tlmSend、fileDownlink、systemResources、comQueue、aggregator、cmdDisp、health、commsBufferManager、`usDht22`（元件規範：偶數 tick 觸發、下一 tick 解碼，每 2 s 一筆；不可接 rg100Hz） |

## 實例與 Base ID
| 實例 | 元件 | Base ID |
|---|---|---|
| rg1kHz／rg100Hz／rg1Hz | Svc.ActiveRateGroup | 0x0100／0x0200／0x0300 |
| rateGroupDriver、chronoTime、timer、systemResources、comDriver | Svc／Drv | 0x0400–0x0800 |
| platform | Mcu.Platform | 0x1000 |
| flightMode | Mcu.FlightMode（`setDeploymentRole(US)`） | 0x1400 |
| nvFlags | Components.NvFlags（RamNvStore） | 0x6100 |
| usMpu6050 | Components.UsMpu6050Sim（真機換 UsMpu6050 0x4200） | 0x4280 |
| usDht22 | Components.UsDht22Sim（真機換 UsDht22 0x4300） | 0x4380 |
| sdLogger | Components.SdLoggerSim（真機換 SdLogger 0x2000；CARD_ROLE=US_LOG） | 0x2080 |

連線：`usMpu6050.mpuOut` → `sdLogger.mpu6050In`；`usDht22.humTempOut` → `sdLogger.humTempIn`。
SIL 開機在 `loadParameters()` 之後自動 `sdLogger.startLogging(US_LOG)`（飛行紀錄器，不靠 GDS 下 START）。

## SIL 實跑結果（3 s）
```
SdLoggerSim READY
FlightMode 未填 FM-20（重啟恢復規則待定）；中止指令來源 PIL 接
Platform 開機 reset=POWER_ON
UsDht22Sim OFF -> INIT -> SELFTEST
UsMpu6050Sim READY
UsMpu6050Sim FAULT reason=3   # 靜置讀值連續 stuckN（預設 50）筆，核心既有行為
UsMcu sdLogger writeCount=51 dropCount=0 sdBusyCount=0 mediaBytes=2632
```
`rg100Hz` 沒有 `RateGroupCycleSlip`／沒有因 SD 寫入造成的逾時；`QueueFullDrop`＝0。本機上看過 `Platform` 的 1 kHz `LoopOverrun` 與偶發 `rg1kHz` `RateGroupCycleSlip`（既有主機時序，DsMcu 也有）。

SIL 預設 MPU 讀值是靜置（accel Z＝2048 LSB＝1 g、gyro＝0），所以大約 0.5 s 有效樣本後會進 `FAULT_STUCK`。要用飛行剖面請呼叫 `MpuSimDevice::setTruth`／`UsMpu6050Sim::setTruth` 餵變化中的原始值。`UsMpu6050Sim` 只把 `valid` 樣本送到 `mpuOut`，避免自測／FAULT 期間刷 `SeqGap`。

## 還沒接的
| 要接的 | 卡在 |
|---|---|
| gimbalServoDriver（0x3200） | Servo 組模組找不到自己的標頭（issue） |
| 上級開傘驅動 | 沒有元件（FM-IN-7） |
| 相機不併入本部署（見 `Deployments/UsCam`） | — |
