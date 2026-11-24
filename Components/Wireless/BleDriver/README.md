# BleDriver（藍牙備用傳輸，BLE）

- Base ID：`0x8800`；Sim `BleDriverSim` `0x8880`（＝+0x80）
- 定位：LoRa（`RfDriver`，0x8400）的**備用傳輸**。主飛行鏈仍是 LoRa；本元件先做可切換的驅動，**不放進任何部署拓樸**，選用時才在 `instances.fpp` 把 `rfDriver` 換成 `bleDriver`。
- 適用場景：桌上／PIL／地面檢測／發射台近距離 checkout（可省一組測試用 LoRa）。飛行遙測與上行維持 LoRa。
- 角色：預設 `PAD_CHECKOUT_LINK`（發射台／地面檢測近距離鏈路）；`AIR_GROUND_LINK` 保留給評估用。

## 為什麼這樣接

```
ComQueue → Framer 幀 ─┬→ RfDriver   ＝ RfDriverCore ＋ LoRa 匯流排（主鏈，未改）
                      └→ BleDriver  ＝ RfDriverCore（同一份原檔）＋ BleLinkCore ＋ IBleHal
上行：BleDriver.dataOut → 解幀 → UplinkAuth（CRC → 防重放 → HMAC → 白名單）→ cmdDisp
```

- **埠集合與 RfDriver 完全相同**（`dataIn`／`dataOut`／`comStatusOut`／`healthOut`／`run`…），拓樸換實例即可。
- **狀態機不重寫**：`RfDriverCore`（OFF→INIT→SELFTEST→READY／FAULT、TX 逾時、連續逾時 FAULT）原封共用；
  BLE 只是實作 `IRfBus` 的另一種匯流排（`BleLinkCore`）。
- **組幀／HMAC／防重放／中止都在傳輸層之上**，本元件不碰、不繞過；BLE 鏈路層加密只是選配，不當安全依據。
- 本元件不含任何金鑰。若之後啟用 BLE 配對，配對資料只從 NVS／外部設定載入（`docs/key-handling.md`）。

## 硬體差異（ESP32 與 STM32WB55）

| | 初代 ESP32（ESP32 DevKit U1，主線） | STM32WB55CGU6（備用 MCU） |
|---|---|---|
| FPP 型號 | `ESP32_INTERNAL_BLE`（**預設**） | `STM32WB55CGU6_INTERNAL_BLE` |
| 協定 | BLE 4.2（LE 1M PHY ＋ DLE） | BLE 5.4（LE 1M／LE 2M） |
| PHY | 僅 `LE_1M`；**不支援** `LE_2M` 與 LE Coded。參數若設 `LE_2M`，`clampBleConfig` 自動回退 `LE_1M`（寫回設定並交給 HAL） | `LE_1M`／`LE_2M`；**不支援** LE Coded |
| ATT_MTU | 23..247（DLE） | 23..247（DLE） |
| TX 功率 | −12..+9 dBm（超出夾住；HAL 可視板級限制再夾） | −40..+6 dBm（超出夾住；HAL 可視板級限制再夾） |
| IBleHal 後端範例 | ESP-IDF NimBLE 或 Bluedroid | STM32_WPAN（SHCI／ACI，無線堆疊跑在 CPU2） |

## HAL（IBleHal）

真機實作由平台提供，本元件只要 7 個非阻塞函式：
`stackStart(cfg)`、`stackAlive()`、`selftest()`、`linkUp()`、`attMtu()`、`notify(片)`、`pollWrite(片)`。

- **ESP-IDF**：用 NimBLE 或 Bluedroid 實作上述介面；堆疊事件回呼只把資料放進 HAL 自己的佇列，`run` 時輪詢。
- **STM32_WPAN**：用 SHCI／ACI 對 CPU2 堆疊；同樣非阻塞、回呼只入列。
- GATT 服務：1 個 notify 特徵（下行）＋1 個 write-without-response 特徵（上行）。UUID 待與平台對齊後再定。
- 拓樸未接 HAL 時用內建空 HAL → INIT 失敗 → FAULT，不會假裝有鏈路。

靜態 RAM 約 **4.6 KB／實例**（TX 佇列 4×512、重組緩衝、RX FIFO 等）。`dataIn` 與 `run` **必須掛在同一個 rate group**（被動元件、佇列無鎖）。

## 切片（BleLinkCore）

- Framer 幀上限 255 B；一個 GATT notification／write 最多 `ATT_MTU − 3` B。每幀切片，每片 1 B 片頭：
  `bit7 FIRST｜bit6 LAST｜bit5..0 片序號（mod 64）`。MTU 247 → 每片 243 B 幀資料（255 B 幀 2 片）；MTU 23 → 19 B（14 片）。
- 收端：序號跳號、沒有 FIRST 的中段片、少了 LAST 就來新 FIRST、收一半斷線 → 整幀作廢並計數，**不交半幀給 Framer**。
- 重組超過最大幀緩衝（512 B）、或完成幀放不進 RX FIFO → 整幀丟並計入 `rxOverflowDrops`（遙測 `RxOverflowDrops`）。
- HAL `pollWrite` 若回傳大於緩衝的 `n`，本層會夾到 `sizeof(frag)`，不越界讀取。

## 掉線與擠塞處理（對齊合約 §4：下行不重傳、丟了計數、永不阻塞）

| 情況 | 行為 | 計數／遙測 |
|---|---|---|
| 沒有中心端連線 | `writeFrame` 回 0 → RfDriverCore 記 TX_FAIL；**不發事件**（沒人連是常態） | `NoLinkDrops`、`TxDroppedCount` |
| HAL／堆疊 TX 緩衝滿 | 剩下的片下一拍續送；外殼 `popFrame` **先檢查** `txPending()`，上一幀沒送完就不出佇列，避免忙時丟幀、SentFrames 對不上 | （安全網）`BusyDrops` |
| 送一半斷線 | 半幀丟掉，不跨連線續送 | `TxAbortedOnLinkLoss`、`LinkDownCount` |
| 收一半斷線／跳號 | 半幀丟掉 | `RxFragErrors` |
| RX 溢位（重組超長或 FIFO 滿） | 整幀丟掉，不交半幀 | `RxOverflowDrops` |
| 協定堆疊沒回應 | `ready()`＝false → `readyDeadline_s` 後 FAULT（與 LoRa 同） | `State` |
| FAULT | 隔 `faultRetry_s` 自動重新 INIT（重送無線設定給 HAL） | `RestartCount`、`BleRestart` 事件 |

`ready()` 的定義是「協定堆疊活著」，**不是**「有連線」，否則沒人連 5 s 就會被 RfDriverCore 判 FAULT。

## 參數（prmDb）

`model`（預設 `ESP32_INTERNAL_BLE`）、`txPower_dBm`（依型號夾取）、`phy`（預設 `LE_1M`；ESP32 設 `LE_2M` 回退）、`attMtu`（23..247）、
`advInterval_ms`、`connInterval_ms`、`supervisionTimeout_ms`；以及與 RfDriver 同義的 `txTimeout_s`、
`initDeadline_s`、`selftestDeadline_s`、`readyDeadline_s`、`txTimeoutLimit`；`faultRetry_s`。
既有參數 ID（0x8800 段）不變。無線設定在下一次 INIT 才送進 HAL。預設值都是起始值，待實機量測後再調。

## 已知限制／待辦

- 與 RfDriver 相同：埠集合沒有 `dataReturnOut`，`dataIn` 的 Fw::Buffer 同步複製、不歸還；接 F´ v4 ComQueue 時兩個驅動一起補。
- `BleDriverSim` 目前只有 FPP 元件＋`BleHalSim`（與 RfDriverSim 做法一致）；要上 SIL 拓樸時再補外殼。
- 地面 BLE 中心端程式（片重組回幀再給 GDS）、GATT UUID 與各平台堆疊設定，等與 MCU／平台對齊後再做。

## 自測

```
bash Components/Wireless/BleDriver/run_host_tests.sh     # 主機快速測
bash tools/verify/fprime_verify.sh --ut --run 3          # 真 F´ v4.4.1（含 BleDriver UT）
```
