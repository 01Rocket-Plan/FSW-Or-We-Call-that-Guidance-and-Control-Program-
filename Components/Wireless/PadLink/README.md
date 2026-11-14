# PadLink：箭↔發射架、發射架↔地面站（Issue #24）

- 負責：無線組（Edward）
- 補 Issue #24 漏掉的兩條鏈路。原本只有「箭↔地面站」（LoRa Ra-02，見 RfDriver／Framer）。

## 三條鏈路總覽

| # | 鏈路 | 實體 | 方向 | 內容 | 程式 |
|---|---|---|---|---|---|
| 1 | 箭 ↔ 地面站 | LoRa Ra-02 433 MHz（SX1278） | 雙向 | 遙測、密碼中止 | `RfDriver`、`Framer`…（既有 PR） |
| 2 | 箭 ↔ 發射架 | **有線臍帶 UART**（箭尾 umbilical，離軌時拔脫） | 雙向 | 箭 → 架：READY／BIT、飛行模式、電池電壓；架 → 箭：架狀態、倒數、點火／推力確認／夾持旗標 | `UmbilicalLink`（DsMcu）↔ `PadController`（PadMcu） |
| 3 | 發射架 ↔ 地面站 | **NRF24L01 2.4 GHz**（不上箭，xmind） | 雙向 | 架上狀態燈；ARM／LAUNCH／ABORT／DISARM／心跳 | `PadController`（PadMcu） |

臍帶是雙向的（Issue #24 審查更正）。誰決定鬆夾持：**方案 A**（Edward 2026-11-08 定案）

- 鬆夾持由**架**決定：推力確認＋全部互鎖過才鬆。
- 箭用 `RocketStatus.ready` **否決**：ready 掉了，倒數中 → 架退回 SAFE；點火後臍帶斷 → HOLD_ABORT。
- 架 → 箭 的 `PadToRocket` 只是狀態回報，不是放行指令。箭不回「放行」，所以箭上 MCU 當掉不會讓夾臂一直夾著燒中的馬達（那是方案 B 的風險）。
- 點火指令仍只走「地面 → 架」，火箭不在點火迴路裡（gate C15 第三輪）。
- 箭收到架狀態後要不要用（例如倒數中預先準備起飛偵測）是 FlightMode 的事，`UmbilicalLink` 只經 `padStatusOut` 轉交。

## 幀格式（兩條新鏈路共用，`PadLinkCodec`）

Framer 的 §2 幀固定開銷 30 B，NRF24L01 單包只有 32 B，塞不進去，所以另訂精簡幀。

| 偏移 | 欄位 | 長度 |
|---|---|---|
| 0 | sync `0xA5 0x5A` | 2 |
| 2 | type | 1 |
| 3 | seq（大端） | 2 |
| 5 | len | 1 |
| 6 | payload | 0–24 |
| 6+len | CRC-16/CCITT-FALSE（涵蓋 type..payload） | 2 |

固定開銷 8 B，最大一幀 32 B ＝ 一包 NRF24。臍帶 UART 用 `StreamDeframer` 逐位元組找 sync，壞幀丟掉計數、自動重新同步。

### 訊息

| type | 名稱 | 方向 | 酬載 |
|---|---|---|---|
| `0x11` | RocketStatus | 箭 → 架 | flightMode、bitFlags、ready、battV_mV、bootCount（7 B） |
| `0x12` | PadToRocket | 架 → 箭 | padState、lastReason、countdown_ds、flags（點火中／推力確認／夾持已鬆／架有收到箭）、padBootId（6 B） |
| `0x21` | PadStatus | 架 → 地面 | padState、lastReason、interlocks、rocketMode、rocketBit、tilt、padV、倒數、lastCmdCounter、lastCmdResult、bootId（19 B） |
| `0x31` | GroundCmd | 地面 → 架 | op、counter、bootId、16 B 截短 HMAC tag（22 B） |

BIT 旗標（E2）：硬擋＝IMU、電池電壓、舵回中、氣壓；警告＝SD、遙測（只回報，可在地面手動放行）。
箭上 `ready`＝硬擋全過、BIT 資料夠新、模式仍是 `PAD_IDLE`／`ARMED`。

## 發射架狀態機（`PadControllerCore`）

```
SAFE ──ARM(簽名, 全綠)──► ARMED ──LAUNCH(簽名)──► COUNTDOWN(30 s 持續評估) ──► IGNITION
  ▲                         │                        │
  └──任一紅燈／ABORT／DISARM─┴────────────────────────┘

IGNITION ──推力持續 thrustConfirm_s（1 kHz 連 20 樣本）──► 再等 releaseMargin_s ──► RELEASED（鬆夾持）
   ├──到 T＝releaseDeadline_s 都沒推力──► HANG_FIRE   （點火關、夾持保持鎖死、不重試）
   ├──等 releaseMargin_s 時推力消失──► HOLD_ABORT／THRUST_LOST（不鬆、繼續夾住）
   ├──有推力但到 T＝releaseDeadline_s 還沒鬆──► HOLD_ABORT／LATE_RELEASE（不鬆了、繼續夾住）
   └──鑰匙／架電／臍帶／導軌異常、ABORT──► HOLD_ABORT（點火關、夾持保持鎖死）
RELEASED ──離軌判定（三層，見下）──► LIFTED_OFF
RELEASED ──clampOpenTimeout_s 內沒讀到夾持開到位──► NO_LIFTOFF／CLAMP_STUCK（點火關、鬆夾持輸出維持、通知人員）
RELEASED ──liftoffTimeout_s 沒離軌──► NO_LIFTOFF／NO_LIFTOFF（夾持已鬆不能再夾回、通知人員）
```

`PadController.tick()` 跑在 **1 kHz 速率群組**（一拍 1 ms；推力檢查在這個群組裡）。狀態幀（`radioOut`、`umbilicalOut`）
仍是 10 Hz，由 F´ 外殼每 100 拍送一次；`PadControllerCore` 本身不假設拍頻，時間都用呼叫端傳入的 `now_s`。
時間比較容差 `kTimeEps_s`＝5e-5 s（一拍的 5%），吸收浮點誤差和排程抖動。

發射許可＝全部 AND：箭體到位 ∧ 點火迴路連續 ∧ 實體鑰匙 ∧ 架傾角 ≤ 5° ∧ 導軌沒卡 ∧ 架電源 ∧ 地面鏈路 ∧ 臍帶鏈路 ∧ 箭體 READY。

| 規則 | 來源 |
|---|---|
| 兩道分開的人工指令（ARM 解軟體鎖、LAUNCH 點火鍵） | SSS「點火前發射台安檢」、C15 |
| 待點火約 30 s，一紅就終止倒數；要重走 ARM → LAUNCH，不會自動恢復 | E1、SSS |
| 地面斷線：ARMED／COUNTDOWN → SAFE；IGNITION 之後不中止（點了只能自己走完） | E1、F9 |
| 先點、確認推力、再鬆夾持；沒鬆時異常就保持鎖死；鬆了就不能再夾回 | F9、H2 第二輪 |
| 點火後沒離台 → 安全狀態並通知，不自動重試 | SSS 破壞性「發射台安檢」 |
| 一次開機只點一次（之後 ARM 一律拒絕） | E3「點火後鎖死」 |
| 實體鑰匙在硬體串聯；軟體再加一道：鑰匙讀回 OFF 時點火輸出一律關 | C15、H4 |
| 起飛後臍帶斷是預期行為，箭上、架上都不報錯 | SSS「箭體、發射架通訊」 |

## 鬆夾持時序與離軌判定（照 Edward PR #52 審查決定實作，2026-11-13）

**推力確認和鬆夾持之間的裕度。** 推力確認之後不在同一拍鬆，而是再等 `releaseMargin_s`；
這段時間每一拍都要仍有推力（鬆架那一拍也要），只要推力消失一拍就 HOLD_ABORT／THRUST_LOST、不鬆。
軟體最小單位是一拍（1e-3 s），比一拍小的裕度等同下一拍；`releaseMargin_s`＝0 就是確認那拍鬆。

**一條時間軸：任務時間 T（Edward 2026-11-14，參考 SpaceX 的 T−／T+）。** T＝0＝點火信號正式發出那一拍。
點火前是 T−：SAFE／ARMED 時 T 停在 −30 s（同一條軸，時鐘還沒走），LAUNCH 後 T−30 → T−0；倒數中止回 SAFE，T 回到 −30 停住。
點火後是 T+，所有事件都記成 T+ 值（遙測 `MissionTime_s`）。程式裡不再用負數當「還沒發生」的標記，改用旗標。

**T＝0 用點火信號，不用推力（Edward 2026-11-14）。** T＝0 是地面站看得到、記得下來的時刻（架的狀態幀變 IGNITION）；
推力什麼時候出現是架上的事。**點火延遲（點火信號 → 推力出現）先當 0**，推力確認、鬆夾持、期限全部從 T＝0 起算。

| 時間（T） | 事件 | 參數 |
|---|---|---|
| T−30 → T−0 | 待點火倒數，一紅就終止 | `countdown_s`＝30 |
| T＝0 | 點火信號發出（點火繼電器通電；這一拍算「還沒推力」） | — |
| T+1e-3 … T+2e-2 | 連 20 個樣本有推力 → T+2e-2 推力確認（推力斷一拍就從那拍重算） | `thrustConfirm_s`＝2e-2 |
| T+2.1e-2 | 再確認一次推力 → 發鬆夾持命令 | `releaseMargin_s`＝1e-3 |
| T+4.1e-2 | 夾持真正打開（假設值，型號定了以後用 HIL 實測） | `clampActuatorLatency_s`＝2e-2 |
| T+5e-2 | 期限；之後還沒鬆 → 不鬆、繼續夾住。有過推力 → HOLD_ABORT／LATE_RELEASE；完全沒推力 → HANG_FIRE（**沒推力就沒人鬆架，故意的**） | `releaseDeadline_s`＝5e-2 |
| 鬆夾持命令後 1e-1 s 內 | 要讀到夾持開到位，否則 NO_LIFTOFF／CLAMP_STUCK | `clampOpenTimeout_s`＝1e-1 |
| 鬆夾持命令後 1 s 內 | 要判定離軌，否則 NO_LIFTOFF | `liftoffTimeout_s`＝1 |

**已知待修：點火延遲（Issue #56）。** 照上表，推力最晚要在 T+3e-2 那一拍出現才鬆得了架
（T+2.9e-2 是最後一個沒推力的樣本 → T+4.9e-2 確認、T+5e-2 鬆）；再晚就 LATE_RELEASE／HANG_FIRE。
實際點火延遲還不知道，**上 HIL 很可能報錯**，SIL 先這樣。要查點火頭規格書、CTI 74F85 搭原廠點火頭的資料，
再用 HIL 量「點火信號 → 荷重元出現推力」，之後改時序。原本的 2 s 點火窗 `ignitionWindow_s` 已刪。

**參數檢查。** `thrustConfirm_s`＋`releaseMargin_s`＋`clampActuatorLatency_s` 必須 ≤ `releaseDeadline_s`
（預設 4.1e-2 ≤ 5e-2），時間參數也必須是有限值且不為負。不合理時 `configure()` 回 false、發 `PadBadParams` 事件，
ARM 以 BAD_PARAMS 拒絕；已在 ARMED／COUNTDOWN 時改成不合理 → 退回 SAFE／BAD_PARAMS。

**架怎麼判定箭離軌了？**（鬆夾持之後，三層）
1. **主判定：離軌開關 `offRail`**，單獨成立就算 LIFTED_OFF（即使沒讀到夾持開到位，因為開到位開關可能壞掉）。
   離軌開關之後可能做成獨立部署的驅動；這裡只把它當成輸入（`PadIoSample.offRail`），不開新部署。
2. **備援：架下荷重元 `load_N`**，低於 `liftoffLoadMin_N`（0.5 N）且持續 `liftoffLoadHold_s`（5e-2 s）就算離軌。
3. **輔助：臍帶逾時**（`umbilicalTimeout_s`）**單獨不算**（可能只是被震斷）；要和「荷重元低於 `liftoffLoadMin_N`」同時成立才算（此時不必等持續時間）。

2、3 只在讀到夾持開到位之後才算：夾持沒開時，箭在拉夾持，荷重元讀值也可能掉下來，不代表箭走了。
三層都不成立、超過 `liftoffTimeout_s` → NO_LIFTOFF，通知人員不要靠近。`thrustOk` 仍是獨立的布林輸入（驅動可以從荷重元換算）。
沒有用「NRF 箭↔地」：NRF24 只在架↔地面，箭上沒有 NRF。

**夾持卡住（CLAMP_STUCK）。** 下令鬆夾持後 `clampOpenTimeout_s` 內沒讀到夾持開到位開關 `clampOpen`，
而且離軌開關也沒觸發 → NO_LIFTOFF，原因 CLAMP_STUCK（發 `PadClampStuck` 事件）。點火早已關，鬆夾持輸出維持 true（不會再鎖回）。
選擇用既有的 NO_LIFTOFF 狀態＋新原因，而不是新增狀態：`PadState` 的數值範圍和 `PadToRocket` 幀格式都不用改，舊的接收端照樣解得開。
CLAMP_STUCK 之後是終態，之後才觸發的離軌開關不會再改成 LIFTED_OFF（寧可多一次「不要靠近」的警告）。
`clampOpenTimeout_s` ≤ 0 表示這套架沒裝開到位開關：不做這項檢查，荷重元備援也不等開到位。

**點火成功後又熄火**（推力確認後、鬆架前推力消失）：照上面的 THRUST_LOST 處理。不另外開新部署，也不改其他組的部署。

### 假設值與假設硬體（Edward 2026-11-13：缺的程式或硬體自己假設、自己訂合理數值）

| 項目 | 假設 |
|---|---|
| 夾持開到位開關 `clampOpen` | 限位開關，開到位＝true；型號、腳位未定 |
| 架下荷重元 `load_N` | 量箭體對架的力（含夾持拉力），單位 N；箭靜置時約等於重量，離軌後趨近 0 |
| `liftoffLoadMin_N` | 0.5 N |
| `liftoffLoadHold_s` | 5e-2 s |
| `clampActuatorLatency_s` | 2e-2 s（繼電器／舵機動作時間，HIL 實測後修正） |
| `releaseMargin_s` | 1e-3 s（1 kHz 一拍） |
| `clampOpenTimeout_s` | 1e-1 s |
| 離軌開關 `offRail` | 由未來的驅動（可能是獨立部署）提供 |

## 指令驗證（`PadCmdGate`）

- 全部指令（ARM、LAUNCH、HEARTBEAT、ABORT、DISARM）都要簽名，deny-by-default；沒接驗證器 → 架子永遠停在 SAFE。
- 地面鏈路「活著」只看簽過名的幀，偽造心跳不能讓倒數在地面斷線後繼續走。
- 防重放：計數器嚴格遞增（RAM）。架子每次開機的 `bootId` 會在 PadStatus 廣播，地面簽名要帶入，所以上次開機錄的舊封包驗不過。
- MAC 輸入：`"01RP-PAD-v1" || bootId || counter_be32 || opcode`；tag＝HMAC-SHA256 前 16 B。
- 金鑰不入庫。`IPadCmdAuth` 預計接 PR #13 的 `HmacSha256`（#13 合併後再接）。

## 已定事項與待辦

1. ~~ABORT／DISARM 要不要簽名？~~ **已定（Edward 2026-11-10，#42 C1）：ABORT 要簽名**（中止是任務終止的標誌，不能讓人偽造）；**DISARM 也簽**（無線組 2026-11-12 審 PR #52）：程式在 ARMED **和 COUNTDOWN** 都接受 DISARM 並退回 SAFE，免簽的 DISARM 在倒數中等於免簽的 ABORT，會繞過 C1；也對齊無線組合約「全部指令預設 HMAC」。Edward 若要 DISARM 免簽，請書面決定，並同時把 DISARM 限定在 ARMED。地面斷線本來就會自動中止，所以 ABORT 要簽名不會「想停停不了」。
2. 門檻：倒數 30 s、**點火延遲當 0、2 s 點火窗已刪（Edward 2026-11-14，Issue #56）**、**推力確認 0.02 s**、**離軌逾時 1 s**（2026-11-10，主機 CTI 74F85-15A 燃燒 0.873 s，#42 C2）、地面鏈路逾時 1 s、臍帶逾時 0.5 s、**箭端收架狀態逾時 1 s**（#42 C3）、架傾角離垂線 5°（＝仰角 85°）、架電源 11 V、**鬆夾持期限 0.05 s**；**已定（Edward 2026-11-13）：數值由無線組自訂**，新增的裕度／荷重元／開到位參數見上方「假設值」表。**1 kHz 速率群組已定**。
3. 硬體型號（假設先行，定了以後修正數值並用 HIL 實測）：推力／離軌用荷重元、夾持開到位開關、離軌開關、seated 開關、傾角計的型號與腳位。
4. `bootId` 來源：建議 PadMcu 的 NvFlags 開機計數（U8 會繞回，256 次開機才重複一次）。
5. 地面站端程式（組 GroundCmd、顯示 PadStatus 燈號）：協定已固定在 `PadLinkCodec.hpp`，地面工具可直接 include。

## 本 PR 不含

- NRF24L01 驅動（SPI）、臍帶 UART 驅動、GPIO／ADC 驅動 → 硬體型號定案後各開一個 PR（Base ID 已預留，見 `docs/conventions.md`）。
- F´ 元件外殼 `.cpp`（`PadController.cpp`／`UmbilicalLink.cpp`）：FPP 已過 `fpp-check`、`fpp-to-cpp`；外殼在拓樸整合時接。
- `FlightMode` 邏輯（CONTRIBUTING 第 7 條）。`UmbilicalLink` 只讀 `ModePort`，架狀態只轉交、不做決定。

## 自測

```
bash Components/Wireless/PadLink/run_host_tests.sh   # 30 組
fpp-check Components/Wireless/PadLink/fpp_stubs/Fw_Svc_stubs.fpp RocketTypes/RocketTypes.fpp \
  Components/Wireless/PadLink/PadLink.fpp Components/Wireless/PadController/PadController.fpp \
  Components/Wireless/UmbilicalLink/UmbilicalLink.fpp
```
