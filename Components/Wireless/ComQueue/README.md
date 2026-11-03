# ComQueue

- 負責：無線組（子Bot1）
- Base ID：`0x8200`
- 範圍：P0..P4 下行優先佇列；`buffOut → framer`；`comStatusIn ← rfDriver`
- 狀態：Core 可 host 自測；F´ 殼待拓樸啟用

## 行為摘要
- 同層 FIFO；跨層 P0→P4 嚴格優先
- P0 滿丟最舊；P1–P4 滿丟最新
- `EVT_QUEUE_FULL_DROP` ≤1 次／100 tick（rg100Hz ≈ 1 s），附累計數
- `comStatusIn` 忙碌時不推進發送
- 不 sleep、不讀系統時間
