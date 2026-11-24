# RfDriver／RfDriverSim 實作說明

## 硬體型號（Edward 設計文件）
1. **主飛行鏈**：LoRa Ra-02（433 MHz），晶片 Semtech SX1278 → FPP `RfHwModel.LORA_RA_02`（本驅動）
2. **備援**：HC-12（SI4438 433 MHz）— **未實作**
3. **地面站／發射台**：NRF24L01（2.4 GHz）— **不上箭**，非本飛行驅動範圍

## 仍開放
- DS／US 頻道分配（`channel` prm）
- Framer `frameMax_B`＝255（SX1278 FIFO 256 B，但 RegPayloadLength 為 8 bit，單包酬載上限 255 B；見 Framer PR #16）

## Base ID
- RfDriver `0x8400`、RfDriverSim `0x8480` — 首席 2026-11-03 核准，main 無衝突。

## 行為
- 時間只來自呼叫端 `now_s`（F´ 時間服務）；不 sleep。
- TX 逾時丟幀＋計數；連續 N 次進 FAULT。
- Sim 掉包在 `writeFrame` 統計；對 Core 回傳 0 視為未送出。
- TX 完成照 LoRa 空中時間（issue #60）：有 TxDone IRQ 來源就等 IRQ，否則數 `ceil(ToA / tickPeriod_us)` tick；
  逾時從空中時間之後再算 `txTimeout_s`。公式見 `LoraTimeOnAir.hpp`（整數 µs，精確值）。

## 仍開放（issue #60）
- SF／BW／CR 已定案：SF7／125 kHz／4:5（Edward）
- 前導碼／CRC／標頭模式的正式值（目前 8／CRC 開／顯式，待 Edward 定）
- 頻段仍在評估（433 MHz 法規問題），定案後可能換頻段／硬體
- RfDriver 掛哪個 rate group（`tickPeriod_us` 目前 10000 ＝ rg100Hz，待定）
