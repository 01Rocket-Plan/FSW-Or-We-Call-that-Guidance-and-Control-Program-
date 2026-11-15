# PadController

- 負責：無線組（Edward）— Issue #24
- 部署：`PadMcu`（發射架 ESP32，不上箭）
- Base ID：`0x8700`
- 範圍：臍帶收箭體狀態、NRF24 收地面指令／送架上狀態、發射許可互鎖、待點火倒數、推力確認後鬆夾持、離軌判斷
- 速率：`schedIn` 接 1 kHz 速率群組；狀態幀 10 Hz（外殼降頻）
- F´ 外殼：`PadController.hpp／.cpp`（只搬資料，邏輯在 Core）。簽名驗證器**未接** → 所有地面指令 `NO_VERIFIER`、停在 SAFE；接 UplinkAuth HMAC 時一起改 bootId（現在固定 0）
- 設計、狀態機、待定事項：見 `../PadLink/README.md`
