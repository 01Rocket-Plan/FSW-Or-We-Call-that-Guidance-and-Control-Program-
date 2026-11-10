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

IGNITION ──推力持續 0.02 s──► RELEASED（鬆夾持）──離軌開關 或 臍帶斷──► LIFTED_OFF
   ├──2 s 點火窗內沒推力──► HANG_FIRE   （點火關、夾持保持鎖死、不重試）
   └──鑰匙／架電／臍帶／導軌異常、ABORT──► HOLD_ABORT（點火關、夾持保持鎖死）
RELEASED ──1 s 沒離軌──► NO_LIFTOFF（夾持已鬆不能再夾回、通知人員）
```

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

## 指令驗證（`PadCmdGate`）

- ARM、LAUNCH、HEARTBEAT 要簽名，deny-by-default；沒接驗證器 → 架子永遠停在 SAFE。
- 地面鏈路「活著」只看簽過名的幀，偽造心跳不能讓倒數在地面斷線後繼續走。
- 防重放：計數器嚴格遞增（RAM）。架子每次開機的 `bootId` 會在 PadStatus 廣播，地面簽名要帶入，所以上次開機錄的舊封包驗不過。
- MAC 輸入：`"01RP-PAD-v1" || bootId || counter_be32 || opcode`；tag＝HMAC-SHA256 前 16 B。
- 金鑰不入庫。`IPadCmdAuth` 預計接 PR #13 的 `HmacSha256`（#13 合併後再接）。

## 待 Edward 定

1. ~~ABORT／DISARM 要不要簽名？~~ **已定（Edward 2026-11-10，#42 C1）：ABORT 要簽名**（中止是任務終止的標誌，不能讓人偽造）；DISARM（ARMED→SAFE，還沒倒數）暫不簽，待 Edward 定。地面斷線本來就會自動中止，所以 ABORT 要簽名不會「想停停不了」。
2. 門檻（SSS 沒有發射架數值，除下列外仍是假設）：倒數 30 s、點火窗 2 s、**推力確認 0.02 s**、**離軌逾時 1 s**（2026-11-10，主機 CTI 74F85-15A 燃燒 0.873 s，#42 C2）、地面鏈路逾時 1 s、臍帶逾時 0.5 s、**箭端收架狀態逾時 1 s**（#42 C3）、架傾角離垂線 5°（＝仰角 85°）、架電源 11 V。
3. 推力確認感測器型號（荷重元？）；離軌開關、seated 開關、傾角計型號與腳位。
4. `bootId` 來源：建議 PadMcu 的 NvFlags 開機計數（U8 會繞回，256 次開機才重複一次）。
5. 地面站端程式（組 GroundCmd、顯示 PadStatus 燈號）：協定已固定在 `PadLinkCodec.hpp`，地面工具可直接 include。

## 本 PR 不含

- NRF24L01 驅動（SPI）、臍帶 UART 驅動、GPIO／ADC 驅動 → 硬體型號定案後各開一個 PR（Base ID 已預留，見 `docs/conventions.md`）。
- F´ 元件外殼 `.cpp`（`PadController.cpp`／`UmbilicalLink.cpp`）：FPP 已過 `fpp-check`、`fpp-to-cpp`；外殼在拓樸整合時接。
- `FlightMode` 邏輯（CONTRIBUTING 第 7 條）。`UmbilicalLink` 只讀 `ModePort`，架狀態只轉交、不做決定。

## 自測

```
bash Components/Wireless/PadLink/run_host_tests.sh   # 18 項
fpp-check Components/Wireless/PadLink/fpp_stubs/Fw_Svc_stubs.fpp RocketTypes/RocketTypes.fpp \
  Components/Wireless/PadLink/PadLink.fpp Components/Wireless/PadController/PadController.fpp \
  Components/Wireless/UmbilicalLink/UmbilicalLink.fpp
```
