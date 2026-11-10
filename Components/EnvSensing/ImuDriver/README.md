# ImuDriver（DS）

- **範圍：** ICM-20948（**單顆**）；1 kHz 非阻塞 `OFF→INIT→SELFTEST→READY/FAULT`
- **Base ID：** `0x4000`；Sim `ImuDriverSim` = `0x4080`
- **FPP module：** `Components`；匯流排埠型別在 `Components_EnvSensing_Bus`（`../Bus/`，issue #34）
- **輸出：** `RocketTypes.ImuSample`，**已是機體系**（X 鼻／Y 右／Z=X×Y，`docs/conventions.md` §2）。
  預設 DCM＝晶片 Z 沿鼻：機體 = (晶片 Z, 晶片 Y, −晶片 X)，墊上靜置 ≈ **(+9.81, 0, 0)**。
  安裝方向不同 → 改 `dcm00…dcm22` 參數。
- **SELFTEST：** bank-1 出廠 OTP（FT = 2620·1.01^(code−1)，FS=0）比對 ST 響應（50%–150%），再做靜置比力／陀螺安靜度
- **故障：** WHO_AM_I／匯流排／自測／卡值 → 直接 `FAULT`（Estimator 20 ms 收不到有效樣本 → `imuValid=false`，Controller 鎖中位）
- **事件：** 只在狀態剛改變時記（`EnterReady`、`EnterFault`、飽和開始），遙測 1 Hz
- **核心：** `imu_core.c`；F´ 外殼 `ImuDriver.cpp`／`ImuDriverSim.cpp`
- **單測：** `../host_test/`；**真 F´ 檢驗：** `tools/verify/fprime_verify.sh Components/EnvSensing/Bus Components/EnvSensing/ImuDriver`

## ICM-20648 去哪了（2026-11-10，Edward #40／#42 D2）
ICM-20648 是「20948 太貴或缺貨時的**替代料**」，不是飛行中的熱備援。替代料不合進 main（不然換料時得動手術），
保留在分支 `EnvSensing/EdwardWang-Claude/icm20648-alt`（＝移除前的雙顆版本）。真的要換料時：
WHO_AM_I 改 `0xE0`、確認 I2C 位址，其餘暫存器兩顆相同（20948 只多了磁力計）。
