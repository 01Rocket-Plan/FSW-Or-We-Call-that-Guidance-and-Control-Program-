# DsCam（ESP32-CAM）

- 負責：相機組（Arthur）／子Bot1
- FPP 宣告：型號 `ESP32_CAM`、角色 `DS_CAM`
- 共用型別：`Components/Camera/Ov3640Driver/CamTypes`（欄位 `camState`／fault／frameCount／model／role；**不**使用 FPP 保留字 `state`）
- 範圍：`Esp32CamCore` + `Esp32CamSimDevice` + F´ 外殼 `Esp32Cam`／`Esp32CamSim`
- 飛控連線：I2C → `ICamLink`；時脈預設 400 kHz；`I2C_ADDR`／`I2C_PIN_SDA`／`I2C_PIN_SCL` 為 placeholder（Anatoly 腳位表）
- 完整錄影：ESP32-CAM 本機 SD（`ILocalSdStore`）；預覽：`IGroundPreviewLink`（不經 ICamLink／遙測）
- 影像埠：`Esp32CamImageOut`（對齊 `Ov3640ImageOut`）：2KB `Fw.Buffer` 分塊，`frameNumber`/`chunkIndex`/`CamChunkFlag`；`FRAME_MAX_BYTES` 截斷不計 `frameCount`、維持 READY
- Rate groups：1 Hz／100 Hz；不碰 1 kHz；不 sleep、不讀系統時間做逾時、不動態配置
- Host：`./run_host_tests.sh`；FPP：`./run_fpp_check.sh`
- 合併順序：先 `camera/ov3640`，再本分支 `camera/esp32cam`

封包提案見 `../CamLinkPacket.md`。
