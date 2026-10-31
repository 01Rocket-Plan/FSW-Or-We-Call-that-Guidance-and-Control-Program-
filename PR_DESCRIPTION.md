# [EnvSensing][子Bot1] IMU drivers (ICM-20948/20648, MPU6050) + Sims

## 摘要
實作環境感測組 IMU 第一次可運行成果：DS `ImuDriver`（ICM-20948 主／ICM-20648 備援）與 US `UsMpu6050`，含 Sim 與 host 狀態機單測。分支：`EnvSensing/子Bot1/imu-driver`。

## 共用型別變更（請副工程師／介面審查）
檔案：`RocketTypes/RocketTypes.fpp`

1. **`Mpu6050Sample` 新增欄位**
   - `accelSat: bool`
   - `gyroSat: bool`
   - 用途：紀錄組事後分析；驅動同時發事件。與 v2 語意對齊，主幹其餘欄位不變。

2. **新增 port**
   - `port Mpu6050Data(s: Mpu6050Sample)`
   - US MPU6050 只送 SdLogger，不進 estimator。

未改動 `ImuSample`／`SampleHeader`／`DrvState`；`ImuData` port 沿用。

## 元件
| 元件 | Base ID | 說明 |
|---|---|---|
| ImuDriver | 0x4000 | 1 kHz 非阻塞 OFF→INIT→SELFTEST→READY/FAULT；機體系輸出；卡值／匯流排／WHO_AM_I 可切備援 |
| ImuDriverSim | 0x4080 | 同核心 + `injectFault`（0 正常／1 卡值／2 gyroSat／3 bus／4 WHO_AM_I／5 accelSat） |
| UsMpu6050 | 0x4200 | 原生軸；含 accelSat／gyroSat |
| UsMpu6050Sim | 0x4280 | 同上 Sim |

邏輯在 `imu_core.c`／`mpu6050_core.c`（無 sleep、不讀系統時間；`t_s` 由呼叫端注入 F´ 時間）。匯流排經 `EnvSensingBus.RegWrite/RegRead`（及 Spi 別名）與 `Drv.I2c`／`Drv.Spi` 抽象埠，部署層接實體驅動。

保守預設 prm：`stuckN=50`、`busErrThreshold=5`、`whoamiFailThreshold=3`；每顆 body←sensor DCM 以標量 prm 給定（預設單位矩陣）。

## 驗證
- `fpp-check`（含 `fpp_stubs`，待 `lib/fprime` 到位後改指正式 Fw／Svc／Drv）
- `host_test/run_all.sh`：狀態轉移、`valid`／`seq`、切備援、飽和旗標、WHO_AM_I 故障

## 缺口／後續
- 完整 F´ AC 產生與部署連線需 MCU 組鎖定 `lib/fprime` 後再跑 `fprime-util`
- ICM bank1 出廠自測值比對目前以靜置比力／陀螺安靜度 SELFTEST 代替，可再加深
- `fpp_stubs/` 僅供本機 fpp-check，不應進飛行映像

## 測試方式
```bash
./host_test/run_all.sh
```
