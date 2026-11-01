# [EnvSensing][子Bot1] IMU drivers (ICM-20948/20648, MPU6050) + Sims

## 摘要
實作環境感測組 IMU 第一次可運行成果：DS `ImuDriver`（ICM-20948 主／ICM-20648 備援）與 US `UsMpu6050`，含 Sim、bank-1 出廠自測、host 狀態機單測，以及對 F´ v4.4.1 標頭的 fpp-to-cpp／單獨編譯檢查。分支：`EnvSensing/imu-driver`（子Bot1）。

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

## 元件（FPP `module Components`）
| 元件 | Base ID | 說明 |
|---|---|---|
| ImuDriver | 0x4000 | 1 kHz 非阻塞 OFF→INIT→SELFTEST→READY/FAULT；bank-1 出廠 ST＋靜置比力；機體系輸出；可切備援 |
| ImuDriverSim | 0x4080 | 同核心 + `injectFault`（0 正常／1 卡值／2 gyroSat／3 bus／4 WHO_AM_I／5 accelSat／6 ST fail） |
| UsMpu6050 | 0x4200 | 原生軸；含 accelSat／gyroSat |
| UsMpu6050Sim | 0x4280 | 同上 Sim |

邏輯在 `imu_core.c`／`mpu6050_core.c`（無 sleep、不讀系統時間；`t_s` 由 F´ 時間服務經元件外殼注入）。匯流排經 `EnvSensingBus.RegWrite/RegRead`（及 Spi 別名）與 `Drv.I2c`／`Drv.SpiWriteRead`。

## 驗證
- `fpp-check`（`fpp_stubs` 對齊 F´ v4.4.1 I2c／Spi 簽名）
- `Components/EnvSensing/host_test/run_all.sh`：狀態轉移、`valid`／`seq`、切備援、飽和、WHO_AM_I、bank-1 ST 通過／失敗切備援
- `Components/EnvSensing/run_fprime_compile_check.sh`：fpp-to-cpp 產生四元件 ComponentAc，並以 zig c++（clang）對 F´ v4.4.1 標頭 `-c -Wall` 編譯通過（含產生碼與外殼／核心）

## 已知缺口
1. 主幹無 `lib/fprime`，未做完整 F´ CMake 建置與連結；編譯檢查是對 v4.4.1 clone 做的。
2. 共用機無 g++，測試／編譯以 zig c++（clang）進行。
3. `fpp_stubs/` 僅供本機 fpp-check，不應進飛行映像。

## 測試方式
```bash
./Components/EnvSensing/host_test/run_all.sh
./Components/EnvSensing/run_fprime_compile_check.sh
```
