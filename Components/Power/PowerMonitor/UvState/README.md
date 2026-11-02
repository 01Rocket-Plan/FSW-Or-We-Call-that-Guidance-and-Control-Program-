# UvStateComp

- 負責：電源組（Zamir）
- 範圍：欠壓分級狀態機（NOMINAL／UV_WARN／UV_SHED／UV_CRITICAL）；只發出 L1／L2 卸載請求埠
- Base ID：`0x6200`（Sim `0x6280`）
- 狀態：v0.1 host-tested core；門檻／dwell／遲滯為 prmDb 占位；F´ shell 待拓撲接線

## 行為摘要
- 進入更嚴重等級需 dwell；退出經遲滯後立即生效
- `valid=false` 或過舊樣本不計入 dwell
- 不下令尾翼鎖定／L3；SEP 與傘通道永不卸載（由 flightMode／actuatorMgr 接手）

## Layout
- `UvState.fpp` / `UvStateComp.*` — F´ shell
- `Core/UvStateCore.*` — host-testable core（內部用 `UvLevel`，避開與 RocketTypes.UvState 名稱衝突）
- `test/UvStateHostTest.cpp`

## Host tests
See `../../run_host_tests.sh`.
