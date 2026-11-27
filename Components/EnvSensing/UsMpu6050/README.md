# UsMpu6050（US）

- **範圍：** MPU6050；僅 SdLogger；感測器原生軸
- **Base ID：** `0x4200`；Sim `UsMpu6050Sim` = `0x4280`
- **FPP module：** `Components`
- **核心：** `mpu6050_core.c`；F´ 外殼 `UsMpu6050.cpp`／`UsMpu6050Sim.cpp`
- **單測：** `../host_test/`
- **Sim 真值：** `MpuSimDevice::setTruth(accel_raw[3], gyro_raw[3])` 餵入原生軸原始 ADC；host 測試 `test/MpuSimDeviceHostTest.cpp`
