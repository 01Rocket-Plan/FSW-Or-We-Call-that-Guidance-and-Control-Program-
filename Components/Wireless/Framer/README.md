# Framer

- 負責：無線組（子Bot1）
- Base ID：`0x8100`
- 範圍：§2 空中幀組幀（同步字／長度／標頭／CRC-32）
- 射頻型號：**LoRa Ra-02**（433 MHz），晶片 **Semtech SX1278**（PR #16 Edward：待射頻型號補上 → 已補）
- `frameMax_B`：預設 **255**。SX1278 FIFO 是 256 B，但單包長度暫存器 `RegPayloadLength` 只有 8 bit，單包酬載上限是 255 B；舊值 256 寫進 8 bit 暫存器會溢位成 0，「剛好 256 B」的幀送不出去。
- 幀頭 26 B＋CRC 4 B＝固定 30 B → 單幀最大酬載 **225 B**

## 幀佈局（大端）
| 偏移 | 欄位 |
|---|---|
| 0 | sync `0xDEADBEEF` (4) |
| 4 | length = payload_B (4) |
| 8 | ver+type (1) |
| 9 | source (1) |
| 10 | APID (2) |
| 12 | seq U32 (4) |
| 16 | bootCount (1) |
| 17 | t_s F64 (8) |
| 25 | valid (1) |
| 26 | payload N |
| 26+N | CRC-32（涵蓋 length..payload）|
