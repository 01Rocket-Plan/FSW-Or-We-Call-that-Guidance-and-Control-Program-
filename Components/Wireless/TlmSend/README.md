# TlmSend

- 負責：無線組（子Bot1）
- Base ID：`0x8000`
- 範圍：頻道取樣 → 邏輯封包（SampleHeader 對齊）→ `comQueue.comIn`
- P1 暫定 10 Hz（`rateDiv=10` @ rg100Hz）
- `t_s` 倒退發 `EVT_TIME_NONMONOTONIC`，仍送出
