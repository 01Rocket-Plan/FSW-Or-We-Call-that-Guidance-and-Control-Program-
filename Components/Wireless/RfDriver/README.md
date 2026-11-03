# RfDriver

- 負責：無線組（子Bot1）
- Base ID：`0x8400`（首席已核准）；Sim `0x8480`
- 硬體型號：`LORA_02` — **LoRa-02**（433 MHz，SX1278-class），**主飛行鏈路**（Edward 設計文件）
- 角色：`AIR_GROUND_LINK`（箭地無線鏈路）
- 握手：`OFF → INIT → SELFTEST → READY／FAULT`（各段截止 prm）
- 連續 `txTimeoutLimit`（預設 3）次 TX 逾時 → FAULT

## 其他射頻（本驅動不實作）
- **HC-12**（SI4438，433 MHz）：備援鏈，**尚未實作**
- **NRF24L01**（2.4 GHz）：地面站／發射台用，**不上箭**

Sim 元件見 `../RfDriverSim/`（不宣告硬體）。
