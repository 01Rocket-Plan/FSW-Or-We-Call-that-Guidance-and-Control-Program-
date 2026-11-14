# DsCam（ESP32-CAM）

- 負責：相機組（Arthur）／子Bot1
- FPP 宣告：型號 `ESP32_CAM`、角色 `DS_CAM`
- 共用型別：`Components/Camera/CamTypes`（FPP 自動碼；2026-11-10 起不准手寫同名 C++ 型別，issue #36）
- 範圍：`Esp32CamCore` + `Esp32CamSimDevice` + F´ 外殼 `Esp32Cam`／`Esp32CamSim`
- 飛控連線：I2C → `ICamLink`；時脈預設 400 kHz；`I2C_ADDR`／`I2C_PIN_SDA`／`I2C_PIN_SCL` 為 placeholder（Anatoly 腳位表）
- 完整錄影：ESP32-CAM 本機 SD（`ILocalSdStore`）；預覽：`IGroundPreviewLink`（不經 ICamLink／遙測）
- 影像埠：`Components.CamImageOut`（與上級相機共用，issue #32）：2KB `Fw.Buffer` 分塊，`frameNumber`/`chunkIndex`/`CamChunkFlag`；`FRAME_MAX_BYTES` 截斷不計 `frameCount`、維持 READY
- Rate groups：1 Hz／100 Hz；不碰 1 kHz；不 sleep、不讀系統時間做逾時、不動態配置

封包提案見 `../CamLinkPacket.md`。
- **驗證**：`tools/verify/fprime_verify.sh --ut Components/Camera/CamTypes Components/Camera/DsCam`（host 測試已搬進 F´ UT／gtest）
