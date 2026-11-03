# RfDriver／RfDriverSim 實作說明

## 開放項（待 Edward 定）
1. 射頻模組型號（FPP `model` 目前 `UNASSIGNED`）
2. `frameMax_B`／MTU／`linkRate_Bps`（屬 framer／預算，與此驅動相關）
3. DS／US 頻道分配（`channel` prm）

## Base ID
- RfDriver `0x8400`、RfDriverSim `0x8480` — 首席 2026-11-03 核准，main 無衝突。

## 行為
- 時間只來自呼叫端 `now_s`（F´ 時間服務）；不 sleep。
- TX 逾時丟幀＋計數；連續 N 次進 FAULT。
- Sim 掉包在 `writeFrame` 統計；對 Core 回傳 0 視為未送出。
