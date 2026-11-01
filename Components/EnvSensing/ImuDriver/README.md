# ImuDriver（DS）

- **範圍：** ICM-20948（主）+ ICM-20648（備援）；1 kHz 非阻塞 `OFF→INIT→SELFTEST→READY/FAULT`
- **Base ID：** `0x4000`；Sim `ImuDriverSim` = `0x4080`
- **FPP module：** `Components`
- **SELFTEST：** bank-1 出廠 OTP（FT = 2620·1.01^(code−1)，FS=0）比對 ST 響應（50%–150%），再做靜置比力／陀螺安靜度
- **核心：** `imu_core.c`；F´ 外殼 `ImuDriver.cpp`／`ImuDriverSim.cpp`
- **單測：** `../host_test/`；F´ 編譯檢查：`../run_fprime_compile_check.sh`
