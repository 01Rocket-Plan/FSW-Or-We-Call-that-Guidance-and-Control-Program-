# WifiDriverSim（WiFi 近距離備用傳輸 Sim）

- Base ID：`0x8980`（＝`WifiDriver` `0x8900` + 0x80）
- 埠集合與 `WifiDriver`／`RfDriver` 相同，可替換拓樸實例；不宣告硬體型號／角色。
- 不得讀取任何 truth／飛行狀態。
- 目前提供 FPP 元件＋可測的 `WifiHalSim`（`IWifiHal` 模擬）。F´ 外殼（繼承 `WifiDriverSimComponentBase`、內含 `RfDriverCore`＋`WifiLinkCore`＋`WifiHalSim`）等決定要上 SIL 拓樸時再補。
- **不接入任何部署拓樸**。主鏈仍是 LoRa。

## WifiHalSim

給主機測試與之後的 SIL 注入情境（數字不是實測，是為了製造情境）：

| 介面 | 用途 |
|---|---|
| `setStackStartOk`／`setStackAlive`／`setSelftestOk` | 注入堆疊啟動失敗、堆疊當掉、自測失敗 |
| `setAssociated`／`setPeerAvailable` | STA 是否已關聯（或 SoftAP 是否有客戶端）、是否有可用對端 |
| `setSendSlotsPerTick` | 每拍最多收幾個資料報；滿了 `send` 回 false，測回壓與續送 |
| `setLoopback` | 送出的資料報回灌到 RX 佇列 |
| `setRssi` | RSSI 遙測 |
| `injectRx` | 注入一個 RX 資料報（可大於最大幀以測溢位） |
| `advance()` | 每拍呼叫：恢復本拍可送槽數 |

`pollRecv` 遇到比緩衝長的資料報時消耗該包、不越界寫入，並回傳實際長度。

## 測試憑證

`WifiDriverHostTest.cpp` 使用明顯假值：SSID `TEST-WIFI-SSID`、密碼 `test-wifi-pass-fake`。這些字串只出現在測試檔。真機由平台從 NVS 或外部設定實作 `IWifiCredStore`。

## 自測

`WifiHalSim` 由 `Components/Wireless/WifiDriver/run_host_tests.sh`（及 F´ UT `Components_Wireless_WifiDriver_ut_exe`）一併編譯執行，無需另跑。
