# RfDriverSim

- Base ID：`0x8480`（＝ RfDriver + 0x80；首席已核准）
- 埠集合與 `RfDriver` 相同，拓樸只換實例
- **不宣告**硬體型號／角色
- prm：`dropRate`／`latencyTicks`／`ber`；不得讀取 truth／飛行狀態
  - `latencyTicks`：上行（地面→箭）位元組 `pushUplink` 後要再過 `latencyTicks` 個 tick 才讀得到（issue #60 第 3 項）
- LoRa 空中時間（issue #60）：公式在 `../RfDriver/LoraTimeOnAir.hpp/.cpp`（Semtech SX1276/77/78/79 datasheet Rev. 4 §4.1.1.7，整數運算）；
  本目錄的 `LoraTimeOnAir.hpp` 只是轉向舊路徑。
  `RfDriverSimDevice` 每送一包就佔線 `ceil(ToA / tickPeriod)` 個 tick，佔線中 `writeFrame` 回 0。
  RfDriverCore 也照同一公式等空中時間才送下一包，所以兩邊參數一致（`configureFrom(RfParams)`）時不會撞到佔線。
  - prm：`loraSf`／`loraBw_Hz`／`loraCr`／`loraPreambleLen`／`loraExplicitHeader`／`loraCrcOn`／`loraLdro`／`tickPeriod_us`
  - 預設與 RfDriver 相同：SF7、125 kHz、4/5（**Edward 定案**）；前導 8、顯式標頭、CRC 開（**待 Edward 定**）；LDRO 自動。
  - 頻段仍在評估（433 MHz 法規問題），定案後可能換頻段／硬體。
  - 例：預設設定下 255 B 一包 = 399.616 ms；rg100Hz 下佔 40 tick。
  - `ready()` 不表示佔線（RfDriverCore 在 READY 時 `ready()==false` 超過 `readyDeadline_s` 會 FAULT），佔線看 `txBusy()`。
