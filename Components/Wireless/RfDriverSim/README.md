# RfDriverSim

- Base ID：`0x8480`（＝ RfDriver + 0x80；首席已核准）
- 埠集合與 `RfDriver` 相同，拓樸只換實例
- **不宣告**硬體型號／角色
- prm：`dropRate`／`latencyTicks`／`ber`；不得讀取 truth／飛行狀態
- LoRa 空中時間（issue #60）：`LoraTimeOnAir.hpp/.cpp` 照 Semtech SX1276/77/78/79 datasheet Rev. 4 §4.1.1.7 公式；
  `RfDriverSimDevice` 每送一包就佔線 `ceil(ToA / tickPeriod)` 個 tick，佔線中 `writeFrame` 回 0（RfDriverCore 記 TX_FAIL）。
  - prm：`loraSf`／`loraBw_Hz`／`loraCr`／`loraPreambleLen`／`loraExplicitHeader`／`loraCrcOn`／`loraLdro`／`tickPeriod_us`
  - 預設＝SX1278 暫存器 reset 值（datasheet §6.4，p.112–114）：SF7、125 kHz、4/5、前導 8、顯式標頭、CRC 關、LDRO 關（AUTO 在 SF7/125k 下＝關）。**無線組可再改**。
  - 例：預設設定下 255 B 一包 = 394.496 ms（CRC 開 399.616 ms）；rg100Hz 下佔 40 tick。
  - `ready()` 不表示佔線（RfDriverCore 在 READY 時 `ready()==false` 超過 `readyDeadline_s` 會 FAULT）；Core 等空中時間再算完成，是 issue #60 第 2 項（無線組）。
