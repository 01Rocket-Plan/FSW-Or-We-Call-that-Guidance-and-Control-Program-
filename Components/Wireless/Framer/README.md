# Framer

- 負責：無線組（子Bot1）
- Base ID：`0x8100`
- 範圍：§2 空中幀組幀（同步字／長度／標頭／CRC-32）
- `frameMax_B`：TBD（待 Edward 定射頻 MTU）；prm 預設 0＝未設上限

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
