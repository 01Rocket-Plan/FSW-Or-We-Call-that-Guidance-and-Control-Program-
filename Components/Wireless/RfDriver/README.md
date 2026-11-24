# RfDriver

- 負責：無線組（子Bot1）
- Base ID：`0x8400`（首席已核准）；Sim `0x8480`
- 硬體型號：`LORA_RA_02` — **LoRa Ra-02**（433 MHz），晶片 **Semtech SX1278**，**主飛行鏈路**（Edward 設計文件）
- 角色：`AIR_GROUND_LINK`（箭地無線鏈路）
- 握手：`OFF → INIT → SELFTEST → READY／FAULT`（各段截止 prm）
- 連續 `txTimeoutLimit`（預設 3）次 TX 逾時 → FAULT

## LoRa 空中時間（issue #60）
- `RfParams`／prm 新增：`spreadingFactor`（`loraSf`）、`bandwidth`（`loraBw`，7.8k..500k）、`codingRate`（`loraCr`）、
  `preambleLen`、`crcOn`、`implicitHeader`、`lowDataRateOptimize`（自動＝Tsym > 16 ms 開）、`tickPeriod_us`。
  SF7／125 kHz／4:5 **已由 Edward 定案**；前導 8／CRC 開／顯式標頭仍**待 Edward 定**；`tickPeriod_us` 預設 10000（rg100Hz，rate group 待定）。
  頻段仍在評估（433 MHz 法規問題），定案後可能換頻段／硬體。
- 參數不合法（SF 不在 6..12、SF6 沒用隱式標頭、BW／CR 編碼錯、tick 0）→ `start()` 直接 FAULT（detail 5）。
- TX 完成：真機接 `setTxDoneSource()`（SX1278 TxDone IRQ）就等 IRQ；沒接就照 `LoraTimeOnAir` 佔 `ceil(ToA / tickPeriod_us)` tick。
  逾時＝超過「空中時間 + `txTimeout_s`」。單包 PL > 255 B 不交給硬體（TX_FAIL）。
- 埠與 `IRfBus`／`IRfSink`／`IRfTxQueue` 都沒改。
- 預設參數 255 B：399.616 ms／幀 → rg100Hz 下 40 tick／幀 ＝ 2.5 幀/s ＝ 637.5 B/s。

## 其他射頻（本驅動不實作）
- **HC-12**（SI4438，433 MHz）：備援鏈，**尚未實作**
- **NRF24L01**（2.4 GHz）：地面站／發射台用，**不上箭**

Sim 元件見 `../RfDriverSim/`（不宣告硬體）。
