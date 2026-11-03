# tools/GDS — LinkLossView（地面缺口／LOS 顯示）

純觀察地面工具核心：依解幀後下行事件流統計每 APID 的 `seq` 缺口與來源 LOS。
**不發送上行、不處理金鑰、不寫回飛控。**

## 行為
1. 以 `seq` + `bootCount` 偵測缺口；U32 無號前進（含 wrap）計入 `gap_count`。
2. 超過可配置 `lossOfSignal_s`（預設 3 s，嚴格 `>`）未收合法幀 → `los=true`。
3. `bootCount` 變化 → 重置該來源 seq 追蹤，避免跨重啟誤報。
4. 亂序／明顯回退不計缺口。

## 測試
```bash
./tools/GDS/run_host_tests.sh
```

落地形式（F´ GDS 外掛 vs 獨立小工具）與首席確認；不挡箭上 PR。
