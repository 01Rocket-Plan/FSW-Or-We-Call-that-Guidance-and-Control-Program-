# Lsm6dsoDriver（DS IMU 備援料：ST LSM6DSO）

> **只在分支 `EnvSensing/Farhad/lsm6dso-alt`，不合 main。** main 的 DS IMU 是 ICM-20948（`../ImuDriver`）。
> 這裡是「想試 LSM6DSO／20948 缺貨時」的替代版本，接口跟 ImuDriver 完全一樣，部署換一行實例就能互換。

- **範圍：** ST LSM6DSO（**單顆**）；1 kHz 非阻塞 `OFF→INIT→SELFTEST→READY/FAULT`
- **Base ID：** 沿用 IMU 槽位 `0x4000`；Sim `Lsm6dsoDriverSim` = `0x4080`（字典、遙測 ID 跟 ImuDriver 對得上）
- **埠／遙測／事件：** 跟 `ImuDriver` 相同 —— `busWrite`／`busRead`（EnvSensingBus.RegWrite/RegRead）、
  `imuOut: RocketTypes.ImuData`、`schedIn`；遙測 `drvState`、`busErrCount`、`stuckCount`、`faultReason`（碼值同 main）；
  事件 `EnterReady`、`EnterFault(reason)`、`GyroSaturated`、`AccelSaturated`（只在狀態剛改變／剛開始飽和時記），遙測 1 Hz＋狀態改變那拍立刻寫。
  `RocketTypes.ImuSample` 沒改。
- **輸出：** 已是機體系（X 鼻／Y 右／Z=X×Y，`docs/conventions.md` §2），由 `dcm00…dcm22` 參數轉換。
  預設 DCM 跟 main 同一個假設：**晶片 Z 沿鼻** → 機體 = (晶片 Z, 晶片 Y, −晶片 X)，墊上靜置 ≈ **(+9.81, 0, 0)**。
  **⚠ LSM6DSO 實際裝在板子上的方向確定後，一定要重設這 9 個 DCM 參數**（不同模組板的晶片方向常跟 ICM 不一樣）。
- **I2C 位址：** 參數 `i2cAddr`，預設 `0x6A`（SDO/SA0 腳接地）；SDO/SA0 接 VDD 時改 `0x6B`。
- **飛行設定：** 加速度 1667 Hz、±16 g（0.488 mg/LSB）；陀螺 1667 Hz、±2000 dps（70 mdps/LSB）；BDU＋IF_INC；
  每拍從 `0x22` 連讀 12 bytes（陀螺 X/Y/Z，再加速度 X/Y/Z，little-endian）。
- **INIT：** `CTRL3_C` SW_RESET → 等 10 ms → `WHO_AM_I(0x0F)` 必須是 `0x6C`（錯 3 次 → FAULT reason 1）→ 等 SW_RESET 位元清掉 → `CTRL3_C = BDU|IF_INC`。
- **SELFTEST（晶片內建自測，照 ST 官方範例 `lsm6dso_self_test.c`）：**
  1. 加速度：52 Hz、±4 g，等 100 ms，丟第一筆、平均 5 筆 → `CTRL5_C ST_XL=正向` → 等 100 ms → 再平均 5 筆，
     每軸 |開−關| 要在 **50–1700 mg**（datasheet DS12140 Table 2）。
  2. 陀螺：208 Hz、±2000 dps，同樣流程（`ST_G=正向`），每軸 |開−關| 要在 **150–700 dps**。
  3. 回飛行設定，等 100 ms，再做墊上靜置檢查（|a| 0.5–1.5 g、角速度 < 1 rad/s，連續 `selftestSamples` 筆；2 s 內不過 → FAULT）。
  開機到 READY 約 **0.8 s**（ICM-20948 版較短）。
- **故障：** WHO_AM_I（1）／匯流排（2）／卡值（3）／自測（4）→ 直接 `FAULT`，不出有效樣本。
- **飽和門檻：** 跟 main 相同（34 rad/s、156 m/s²）。LSM6DSO ±2000 dps 檔的原始上限其實到 ≈ 2294 dps（40 rad/s），
  所以 34 rad/s ≈ 1948 dps 會在「額定量程快到頂」時就標飽和，跟 ICM 的意義一致。
- **核心：** `lsm6dso_core.c`（純 C，跟 `imu_core` 符號不衝突，兩個可同時編進來）；暫存器表 `Lsm6dsoRegs.h`；
  F´ 外殼 `Lsm6dsoDriver.cpp`／`Lsm6dsoDriverSim.cpp`；Sim 晶片 `Lsm6dsoSimDevice.hpp`（注入碼同 ImuSimDevice，另加 1＝卡值；有 ±1 LSB 抖動，SIL 不會被誤判卡值）。
- **單測：** `../host_test/test_lsm6dso_core.cpp`（`run_all.sh` 會跑）；**真 F´ 檢驗：**
  `tools/verify/fprime_verify.sh Components/EnvSensing/Bus Components/EnvSensing/Lsm6dsoDriver`，整個 repo＋SIL：`tools/verify/fprime_verify.sh --run 3`。
- **資料來源：** 暫存器位址、位元、ODR／FS 編碼、靈敏度：ST 官方驅動 `github.com/STMicroelectronics/lsm6dso-pid`（`lsm6dso_reg.h/.c`）；
  自測極限：ST datasheet DS12140 Rev 3 Table 2；自測步驟：ST `STMems_Standard_C_drivers/lsm6dso_STdC/examples/lsm6dso_self_test.c`。
- **型號宣告：** main 目前沒有 `EnvSensorTypes`（型號／角色 enum），所以 LSM6DSO 的晶片常數（WHO_AM_I 0x6C、I2C 0x6A/0x6B）放在本元件 `lsm6dso_core.h`；
  等 `EnvSensorTypes` 進 main 再補 `LSM6DSO` 型號。

## 怎麼跟 ICM-20948 互換
| 要改的 | main（ICM-20948） | 本分支（LSM6DSO） |
|---|---|---|
| 真機實例 | `Components.ImuDriver base id 0x4000` | `Components.Lsm6dsoDriver base id 0x4000` |
| SIL 實例（DsMcu） | （main 目前用 MCU 組佔位 `silImuFeed`） | `imuDriverSim: Components.Lsm6dsoDriverSim base id 0x4080` |
| `i2cAddr` 參數 | 0x69 | 0x6A（或 0x6B） |
| `dcm**` 參數 | 依 ICM 安裝 | **依 LSM6DSO 實際安裝重設** |
| 其他元件（Estimator 等） | 不用改 | 不用改 |
