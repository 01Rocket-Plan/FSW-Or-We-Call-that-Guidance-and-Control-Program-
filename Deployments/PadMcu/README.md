# PadMcuDeployment

發射架控制板（ESP32，獨立板），**不上箭**。Issue #24 新增的第五份部署。
本版是 **Linux SIL**（F´ v4.4.1）；ESP32 版跟 MCU 組 ESP32 平台 PR 一起做。

## 拓樸
| 實例 | 元件 | Base ID | 速率 |
|---|---|---|---|
| `padController` | `Components.PadController`（F´ 外殼 → `PadControllerCore`） | 0x8700 | rg1kHz（狀態幀／遙測內部降到 10 Hz） |
| `rg1kHz`／`rg1Hz` | `Svc.ActiveRateGroup` | 0x0100／0x0300 | — |
| rateGroupDriver、chronoTime、timer、systemResources、comDriver | Svc／Drv | 0x0400–0x0800 | — |
| CdhCore／ComCcsds／FileHandling | F´ 子拓樸（指令、事件、遙測、參數、GDS） | F´ 預設 | 1 Hz |

## 跑 SIL
```bash
tools/verify/fprime_verify.sh --run 3 Components/Wireless/PadLink Components/Wireless/UmbilicalLink \
  Components/Wireless/PadController Deployments/PadMcu
```
SIL 結果（3 s）：開機、載參數、停在 `SAFE`。沒有 PadController 事件是**正常的**：沒有狀態變化就不發事件
（verify 腳本在「沒印出任何事件」時結束碼會是 1，這不是當機；直接跑執行檔結束碼為 timeout 的 124）。
遙測 `State`／`Interlocks`／`MissionTime_s`（−30 s，hold）10 Hz 更新，可從 GDS 看。

## SIL 為什麼不會點火（照實寫）
- **簽名驗證器沒接**（`IPadCmdAuth = nullptr`）→ `PadCmdGate` deny-by-default，所有地面指令回 `NO_VERIFIER`。
  飛行金鑰不在 repo（`docs/key-handling.md`）；之後接 UplinkAuth HMAC 時，`bootId` 一起改成開機產生（現在固定 0）。
- **沒接 `ioGet`** → 架上讀值全 false／0 → 互鎖全紅。
- **沒接 `actOut`** → 就算有輸出也沒有點火繼電器／夾持。

## 鏈路（規劃）
- 箭 ↔ 架：有線臍帶 UART 雙向（箭送 READY／BIT，架回送架狀態），起飛時拔脫（預期中斷，不報錯）
- 架 ↔ 地面：NRF24L01 2.4 GHz
- 點火指令只走地面 → 架；火箭不在點火迴路裡（gate C15）

## 還沒接的（驅動 PR）
| 要接的 | 說明 |
|---|---|
| `nrf24Driver.recv → padController.radioIn`；`padController.radioOut → nrf24Driver.send` | NRF24L01（SPI），Nrf24Driver 0x8600 預留 |
| `umbilicalUart.recv → padController.umbilicalIn`；`padController.umbilicalOut → umbilicalUart.send` | 臍帶 UART |
| `padController.ioGet → padIo`；`padController.actOut → padIo` | GPIO／ADC（seated、迴路連續性、鑰匙讀回、離軌開關、推力、荷重元、傾角、架電源）、點火繼電器與夾持 |
| 簽名驗證器 | UplinkAuth HMAC＋本機金鑰 |

## 電源
- 12 V 繼電器電源給點火與夾持（xmind「DC12V／X0122」）

## 安全
- 架上不准裝真馬達點火直到 GATE-2（gate H11）；地面先用假負載測聯鎖與連續性。
- 實體鑰匙在硬體上串聯點火迴路；軟體只是第二道。
