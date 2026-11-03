# RfDriver／RfDriverSim 實作說明

## 硬體型號（Edward 設計文件）
1. **主飛行鏈**：LoRa-02（433 MHz，SX1278-class）→ FPP `RfHwModel.LORA_02`（本驅動）
2. **備援**：HC-12（SI4438 433 MHz）— **未實作**
3. **地面站／發射台**：NRF24L01（2.4 GHz）— **不上箭**，非本飛行驅動範圍

## 仍開放
- DS／US 頻道分配（`channel` prm）
- Framer `frameMax_B`＝256（對齊 LoRa-02 256 Byte buffer；見 Framer PR）

## Base ID
- RfDriver `0x8400`、RfDriverSim `0x8480` — 首席 2026-11-03 核准，main 無衝突。

## 行為
- 時間只來自呼叫端 `now_s`（F´ 時間服務）；不 sleep。
- TX 逾時丟幀＋計數；連續 N 次進 FAULT。
- Sim 掉包在 `writeFrame` 統計；對 Core 回傳 0 視為未送出。
