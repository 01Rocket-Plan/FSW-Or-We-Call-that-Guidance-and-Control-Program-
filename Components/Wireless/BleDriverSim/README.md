# BleDriverSim（藍牙備用傳輸 Sim）

- Base ID：`0x8880`（＝`BleDriver` `0x8800` + 0x80）
- 埠集合與 `BleDriver`／`RfDriver` 相同，可替換拓樸實例；不宣告硬體型號／角色。
- 不得讀取任何 truth／飛行狀態。
- 目前提供 FPP 元件＋可測的 `BleHalSim`（`IBleHal` 模擬）。F´ 外殼（繼承 `BleDriverSimComponentBase`、內含 `RfDriverCore`＋`BleLinkCore`＋`BleHalSim`）等決定要上 SIL 拓樸時再補。

## BleHalSim

給主機測試與之後的 SIL 注入情境（數字不是實測，是為了製造情境）：

| 介面 | 用途 |
|---|---|
| `setFault(START_FAIL／SELFTEST_FAIL／STACK_DEAD)` | 注入堆疊啟動失敗、自測失敗、堆疊當掉 |
| `setLinkUp`／`setAutoLink(dropEvery, reconnectAfter)` | 連線／斷線，或按拍數自動斷線重連 |
| `setNegotiatedMtu(23..247)` | 協商後 ATT_MTU（設 23 可測最小 MTU 切片） |
| `setTxSlotsPerTick` | 每拍最多收幾個 notification；滿了 `notify` 回 false，測回壓 |
| `peerWriteFragment`／`peerWriteFrame` | 模擬中心端寫入一片或整幀（同一套 1 B 片頭） |
| `advance()` | 每拍呼叫：清 TX 槽、跑自動斷線／重連 |

下行：把收到的 notification 依片頭重組，可用 `peerFrames`／`lastPeerFrame` 檢查。

## 自測

`BleHalSim` 由 `Components/Wireless/BleDriver/run_host_tests.sh`（及 F´ UT `Components_Wireless_BleDriver_ut_exe`）一併編譯執行，無需另跑。
