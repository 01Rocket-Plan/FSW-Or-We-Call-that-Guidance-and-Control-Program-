# RfDriver

- 負責：無線組（子Bot1）
- Base ID：`0x8400`（首席已核准）；Sim `0x8480`
- 硬體型號：`UNASSIGNED`（射頻型號 TBD，**待 Edward 定**）
- 角色：`AIR_GROUND_LINK`（箭地無線鏈路）
- 握手：`OFF → INIT → SELFTEST → READY／FAULT`（各段截止 prm）
- 連續 `txTimeoutLimit`（預設 3）次 TX 逾時 → FAULT

Sim 元件見 `../RfDriverSim/`（不宣告硬體）。
