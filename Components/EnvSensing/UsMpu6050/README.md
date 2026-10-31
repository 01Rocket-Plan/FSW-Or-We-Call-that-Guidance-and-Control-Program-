# UsMpu6050（Base `0x4200`）／UsMpu6050Sim（`0x4280`）

- **負責：** 環境感測系統組子Bot1
- **範圍：** US MPU6050，僅記錄；**原生軸**（`accel_sensor_mps2`／`gyro_sensor_rads`）
- **輸出：** `RocketTypes.Mpu6050Data`（含 `accelSat`／`gyroSat`）
- **匯流排：** `EnvSensingBus` + `Drv.I2c`
- **核心：** `Mpu6050Core`；host 單測見 `host_test/`
