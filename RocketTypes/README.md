# RocketTypes（共用型別與埠）

介面合約 v0.1＋勘誤 v0.2。要改先提首席工程師（CONTRIBUTING 第 8 條）。

## FlightModeId（2026-11-09 對齊 SSS）

來源：`01Rocket_SoftWare_Design/SSS_Mission/sss/fsw/flight_mode.py`（Edward 定案：以 SSS 為準）。

| 值 | 名稱 | 哪一級 | 白話 |
|---:|---|---|---|
| 0 | `PAD_IDLE` | DS | 保險沒解，閒置 |
| 1 | `ARMED` | DS | 已解保險，等 FIRE |
| 2 | `LIFTOFF` | DS | 離台當拍，閂 T0；下一拍進被動助推 |
| 3 | `BOOST_PASSIVE` | DS | 助推初段，舵鎖中位 |
| 4 | `BOOST_ACTIVE` | DS | 速度夠了，舵閉環 |
| 5 | `COAST` | DS | 燃盡滑翔，分離抑制窗計時 |
| 6 | `SEPARATED` | DS | 已下令分離，等分離開關 |
| 7 | `LS_DESCENT` | DS | 分離後下級下墜，鎖舵 |
| 8 | `LS_CHUTE` | DS | 下級開傘 |
| 9 | `LS_LANDED` | DS | 下級著陸 |
| 10 | `HANGFIRE` | DS | FIRE 後沒離台，地面安全態 |
| 11 | `SEP_FAIL` | DS | 下令後分離開關仍通；側傘、禁上級傘 |
| 12 | `ABORT` | DS | D12／密碼中止 |
| 13 | `US_WAIT` | US | 分離前等待 |
| 14 | `US_COAST` | US | 已分離，上級開傘計時中 |
| 15 | `US_CHUTE` | US | 上級開傘 |
| 16 | `US_LANDED` | US | 上級著陸 |
| 17 | `US_CHUTE_INHIBIT` | US | 假分／SEP_FAIL／ABORT：禁上級傘 |

### 這次改了什麼
- **新增** `HANGFIRE`、`ABORT`、`US_WAIT`、`US_CHUTE_INHIBIT`（SSS 有、舊列舉沒有）。
- **移除** `LS_LEGS`：SSS 腳架站 `CP-LEGS-*` 為 DISABLED（腳架設計未定案）。定案後加在 17 之後，不插隊。
- **數值寫死**：`PAD_IDLE=0`、`ARMED=1` 與舊版相同；`UmbilicalLinkCore.hpp` 的 `kModePadIdle`／`kModeArmed` 不必改。其餘數值都變了，但目前 repo 裡沒有程式依賴它們。

### 不在這次範圍
- `Components/Wireless/UplinkAuth/whitelist/WhitelistGate.hpp` 有自己的 `FlightModeId`（`MODE_PAD`、`MODE_BOOST_AFTER`…），是白名單用的粗分類，跟這張表不是同一個東西，沒改。之後要對齊再開 issue。
