# ComQueue

- 負責：無線組（子Bot1）
- Base ID：`0x8200`
- 範圍：P0..P4 下行優先佇列；`buffOut → framer`；`comStatusIn ← rfDriver`
- **passive**＋**guarded** `comIn[5]`（多 rate group 入列，元件 mutex）；`run`＝sync，僅 Core 0 rg100Hz
- 每 `run` 最多 `maxPktsPerRun`（預設 4，≥1）幀；不一次排空
- Viktor：若 rg 預算爆掉再改 active

## 行為摘要
- 同層 FIFO；跨層 P0→P4 嚴格優先
- P0 滿丟最舊；P1–P4 滿丟最新
- `EVT_QUEUE_FULL_DROP` ≤1 次／100 tick，附累計數
- `comStatusIn` 忙碌時不推進
- Core 無 sleep／無鎖；執行緒安全靠 F´ guarded mutex（host 測試用 `std::mutex` 模擬）
