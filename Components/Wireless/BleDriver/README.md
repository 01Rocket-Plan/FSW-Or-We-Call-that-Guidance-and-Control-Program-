# BleDriver（BLE 數傳驅動，LoRa 的備用傳輸）

- Base ID：`0x8800`（**提案，待首席核准**）；Sim `BleDriverSim` `0x8880`（＝+0x80）
- 硬體型號：`STM32WB55CGU6_INTERNAL_BLE` — STM32WB55CGU6 內建 2.4 GHz BLE（CPU2／Cortex-M0+ 跑 ST 無線堆疊）
- 角色：預設 `PAD_CHECKOUT_LINK`（發射台／地面檢測近距離鏈路）；`AIR_GROUND_LINK` 保留給評估用
- 主飛行鏈仍是 LoRa Ra-02（`RfDriver`，0x8400）；本元件**不放進任何部署拓樸**，選用時才換實例
- 可行性評估：`BLE_FEASIBILITY.md`（PR 附件，不入庫）

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

## 切片（BleLinkCore）

- Framer 幀上限 255 B；一個 GATT notification／write 最多 `ATT_MTU − 3` B。每幀切片，每片 1 B 片頭：
  `bit7 FIRST｜bit6 LAST｜bit5..0 片序號（mod 64）`。MTU 247 → 每片 243 B 幀資料（255 B 幀 2 片）；MTU 23 → 19 B（14 片）。
- 收端：序號跳號、沒有 FIRST 的中段片、少了 LAST 就來新 FIRST、收一半斷線 → 整幀作廢並計數，**不交半幀給 Framer**。

## 掉線處理（對齊合約 §4：下行不重傳、丟了計數、永不阻塞）

| 情況 | 行為 | 計數／遙測 |
|---|---|---|
| 沒有中心端連線 | `writeFrame` 回 0 → RfDriverCore 記 TX_FAIL；**不發事件**（沒人連是常態） | `NoLinkDrops`、`TxDroppedCount` |
| CPU2 TX 緩衝滿 | 剩下的片下一拍續送；送完前新幀算忙、丟 | `BusyDrops` |
| 送一半斷線 | 半幀丟掉，不跨連線續送 | `TxAbortedOnLinkLoss`、`LinkDownCount` |
| 收一半斷線／跳號 | 半幀丟掉 | `RxFragErrors` |
| CPU2 堆疊沒回應 | `ready()`＝false → `readyDeadline_s` 後 FAULT（與 LoRa 同） | `State` |
| FAULT | 隔 `faultRetry_s` 自動重新 INIT（重送無線設定給 CPU2） | `RestartCount`、`BleRestart` 事件 |

`ready()` 的定義是「CPU2 堆疊活著」，**不是**「有連線」，否則沒人連 5 s 就會被 RfDriverCore 判 FAULT。

## 參數（prmDb）

`txPower_dBm`（夾到 −40..+6）、`phy`（LE_1M／LE_2M；WB55 不支援 LE Coded）、`attMtu`（23..247）、
`advInterval_ms`、`connInterval_ms`、`supervisionTimeout_ms`；以及與 RfDriver 同義的 `txTimeout_s`、
`initDeadline_s`、`selftestDeadline_s`、`readyDeadline_s`、`txTimeoutLimit`；`faultRetry_s`。
無線設定在下一次 INIT 才送進 CPU2。預設值都是起始值，待實機量測後再調。

## HAL（IBleHal）— 給 MCU 平台

真機實作放在 STM32WB55 平台（STM32_WPAN：SHCI／ACI），本元件只要 7 個非阻塞函式：
`stackStart(cfg)`、`stackAlive()`、`selftest()`、`linkUp()`、`attMtu()`、`notify(片)`、`pollWrite(片)`。
CPU2 事件回呼（連線／斷線／MTU 交換／GATT write）只把資料放進 HAL 自己的佇列，`run` 時輪詢。
GATT 服務：1 個 notify 特徵（下行）＋1 個 write-without-response 特徵（上行）。UUID 待定。
CPU2 韌體需要 ST 的 BLE 堆疊二進位（建議 `stm32wb5x_BLE_Stack_full_fw.bin`；要擴充廣播才用 `_full_extended_`）。
拓樸未接 HAL 時用內建空 HAL → INIT 失敗 → FAULT，不會假裝有鏈路。

## 已知限制／待辦

- 與 RfDriver 相同：埠集合沒有 `dataReturnOut`，`dataIn` 的 Fw::Buffer 同步複製、不歸還；接 F´ v4 ComQueue 時兩個驅動一起補。
- `BleDriverSim` 目前只有 FPP 元件＋`BleHalSim`（與 RfDriverSim 做法一致）；要上 SIL 拓樸時再補外殼。
- 中心端（地面）需要一個 BLE central（筆電／手機／另一塊 WB55 USB dongle）＋把片重組回幀的地面程式。

## 自測

```
bash Components/Wireless/BleDriver/run_host_tests.sh     # 主機快速測（10 組）
bash tools/verify/fprime_verify.sh --ut --run 3          # 真 F´ v4.4.1（含 BleDriver UT）
```
