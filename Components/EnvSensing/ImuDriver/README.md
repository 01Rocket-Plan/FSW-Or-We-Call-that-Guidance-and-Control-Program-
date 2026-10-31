# ImuDriver（Base `0x4000`）／ImuDriverSim（`0x4080`）

- **負責：** 環境感測組子Bot1
- **範圍：** DS ICM-20948（主）+ ICM-20648（備援）；1 kHz 非阻塞握手 `OFF→INIT→SELFTEST→READY/FAULT`
- **輸出：** `RocketTypes.ImuData`（機體系 `acc_mps2`／`gyro_rads`，`gyroSat`）；`valid` 僅 READY
- **匯流排：** `EnvSensingBus.RegWrite/RegRead`（及 Spi 別名）+ `Drv.I2c`／`Drv.Spi`；部署層接到實體驅動
- **參數：** `stuckN`（預設 50）、`busErrThreshold`（5）、`whoamiFailThreshold`（3）、每顆 DCM、飽和門檻
- **核心：** `ImuCore`（可 host 單測）；元件殼呼叫 `tick(t_s,…)`，`t_s` 只來自 F´ 時間服務
- **Sim：** `injectFault`：0 正常／1 卡值／2 gyroSat／3 匯流排／4 WHO_AM_I／5 accelSat
