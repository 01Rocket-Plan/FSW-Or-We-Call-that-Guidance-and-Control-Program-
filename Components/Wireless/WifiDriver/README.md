# WifiDriver（WiFi 近距離備用傳輸）

- Base ID：`0x8900`；Sim `WifiDriverSim` `0x8980`（＝+0x80）
- 定位：LoRa（`RfDriver`，0x8400）的**近距離備用傳輸**。主飛行鏈仍是 LoRa；本元件先做可切換的驅動，**不放進任何部署拓樸**，選用時才在 `instances.fpp` 增加實例。本 PR **不取消、不修改 LoRa**。
- 適用場景：約 100 m 視距等級的桌上／PIL／初期試飛備用。長距離主任務仍走 LoRa。
- 同一套驅動是通用資料報管道：將來可用於箭↔地面（承載既有 Framer 幀）或發射架↔地面（承載既有 PadLink 精簡幀）。本元件**不解析或改寫幀內容**。

## 為什麼這樣接

```
ComQueue → Framer 幀 ─┬→ RfDriver    ＝ RfDriverCore ＋ LoRa 匯流排（主鏈，未改）
                      ├→ WifiDriver  ＝ RfDriverCore（同一份原檔）＋ WifiLinkCore ＋ IWifiHal
                      └→ BleDriver   ＝ 既有近距離備用（未改）
上行：WifiDriver.dataOut → 解幀 → 既有上行鑑權元件 → cmdDisp
```

- **埠集合與 RfDriver／BleDriver 完全相同**（`dataIn`／`dataOut`／`comStatusOut`／`healthOut`／`run`…），拓樸換實例即可。
- **狀態機不重寫**：`RfDriverCore`（OFF→INIT→SELFTEST→READY／FAULT、TX 逾時、連續逾時 FAULT）原封共用；WiFi 只是實作 `IRfBus` 的另一種匯流排（`WifiLinkCore`）。
- **組幀與上行鑑權都在傳輸層之上**，本元件不碰、不繞過。
- 本元件不含任何金鑰或真實 WiFi 憑證。SSID／密碼只從 `IWifiCredStore`（NVS／外部設定）載入；測試用明顯假值 `TEST-WIFI-SSID`／`test-wifi-pass-fake`。密碼不寫進日誌、事件或遙測。

## 硬體差異

| | 初代 ESP32（ESP32 DevKit U1，主線） | STM32WB55CGU6（備用 MCU） |
|---|---|---|
| FPP 型號 | `ESP32_INTERNAL_WIFI`（**預設**） | `UNSUPPORTED`（沒有 WiFi） |
| 無線 | 內建 2.4 GHz WiFi（802.11 b/g/n）與 BLE（共用同一顆無線電） | **無 WiFi**；`init` 失敗 → FAULT，並發 `WifiUnsupportedHw` |
| TX 功率 | 夾取 8–20 dBm（超出夾住；HAL 可視板級限制再夾） | 不啟動堆疊 |
| IWifiHal 後端 | ESP-IDF `esp_wifi`＋lwIP UDP | 不適用 |

初代 ESP32 的 WiFi 與 BLE 共用 2.4 GHz 電波，建議開啟 ESP-IDF software coexistence。本驅動不要求與 BLE 同時發送同一幀。

## HAL（IWifiHal）

真機實作由平台提供，本元件只要非阻塞函式：
`stackStart(cfg)`、`stackAlive()`、`selftest()`、`linkUp()`、`send(資料報)`、`pollRecv(資料報)`，以及可選的 `rssi_dBm()`。

- 堆疊事件回呼只把資料放進 HAL 自己的佇列，`run`／`service` 時輪詢。
- `pollRecv` 若收到比緩衝更長的資料報：消耗該包、不越界寫入，並回傳實際長度，本層計入 `rxOverflowDrops`、不交半幀。
- 拓樸未接 HAL 時用內建空 HAL → INIT 失敗 → FAULT，不會假裝有鏈路。
- 憑證缺失或為空同樣 INIT 失敗 → FAULT。

`dataIn` 與 `run` **必須掛在同一個 rate group**（被動元件、佇列無鎖）。

## 傳輸與組網

- **承載：UDP**，一個資料報恰好一幀，不加私有切片頭。
- **預設模式：STA**（連到外部 AP）。`SOFT_AP` 供沒有路由器的桌上測試。
- 參數提供本機／對端 UDP 埠（預設 4242）與對端 IPv4。`peerIp` 以網路位元序 U32 存放（位元 31–24＝第一段；例如 127.0.0.1＝`0x7F000001`）。
- `learnPeerFromRx`：為真時，從最近一次成功收到的來源更新對端。
- `ready()`＝「協定堆疊活著」，**不是**「已關聯」。未關聯是常態，不會因此進 FAULT。

## 掉線與擠塞處理（下行不重傳、丟了計數、永不阻塞）

| 情況 | 行為 | 計數／遙測 |
|---|---|---|
| 沒有關聯或沒有對端 | `writeFrame` 回 0 → RfDriverCore 記 TX_FAIL；**不發事件** | `NoLinkDrops`、`TxDroppedCount` |
| HAL／socket TX 緩衝滿 | 本幀留在 pending，下一拍續送；外殼 `popFrame` **先檢查** `txPending()`，上一幀沒送完就不出佇列 | （安全網）`BusyDrops` |
| 送一半斷線 | 半幀丟掉，不跨關聯續送 | `TxAbortedOnLinkLoss`、`LinkDownCount` |
| RX 溢位（資料報超長或 FIFO 滿） | 整幀丟掉，不交半幀 | `RxOverflowDrops` |
| 協定堆疊沒回應 | `ready()`＝false → `readyDeadline_s` 後 FAULT（與 LoRa 同） | `State` |
| FAULT | 隔 `faultRetry_s` 自動重新 INIT | `RestartCount`、`WifiRestart` 事件 |
| 硬體沒有 WiFi | INIT 失敗 → FAULT | `WifiUnsupportedHw` |

## 參數（prmDb）

`model`（預設 `ESP32_INTERNAL_WIFI`）、`role`、`mode`（預設 STA）、`channel`、`txPower_dBm`、`localPort`、`peerIp`、`peerPort`、`learnPeerFromRx`，以及與 RfDriver 同義的 `txTimeout_s`、`initDeadline_s`、`selftestDeadline_s`、`readyDeadline_s`、`txTimeoutLimit`、`faultRetry_s`。SSID／密碼**不是** FPP 參數。無線設定在下一次 INIT 才送進 HAL。

## 已知限制／待辦

- 與 RfDriver／BleDriver 相同：埠集合沒有 `dataReturnOut`，`dataIn` 的 Fw::Buffer 同步複製、不歸還；接 F´ v4 ComQueue 時一起補。
- `WifiDriverSim` 目前只有 FPP 元件＋`WifiHalSim`（與 BleDriverSim 做法一致）；要上 SIL 拓樸時再補外殼。
- 本輪不新增多載體切換器，也不把本驅動接到 DsMcu／PadMcu 拓樸（須另行核准）。
- 地面 UDP 接收小工具（`WifiUdpTap`）尚未實作。
- 真機 ESP-IDF HAL 由平台後續提供。

## 自測

```
bash Components/Wireless/WifiDriver/run_host_tests.sh     # 主機快速測
CXXFLAGS="-fsanitize=address,undefined -g" bash Components/Wireless/WifiDriver/run_host_tests.sh
bash tools/verify/fprime_verify.sh --ut --run 3          # 真 F´ v4.4.1（含 WifiDriver UT）
```
