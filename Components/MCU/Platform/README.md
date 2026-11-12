# Platform（踢狗、迴圈監測、重開機原因）

- 負責：MCU 組（EdwardWang+Claude）
- 部署：DsMcu、UsMcu
- Base ID：`0x1000`（SilImuFeed `0x1080`）
- 速率：**rg1kHz 的最後一個成員**
- 狀態：v0.1（SIL）；主機測試 2 站 PASS；F´ v4.4.1 編譯 0 警告

## 一句話
1 kHz 鏈每拍跑完才踢一次狗；鏈上任何元件卡住，這裡就不會被叫到，硬體看門狗就會重開（ESP32）。

## 做什麼
| 功能 | SIL（本 PR） | ESP32（下一個 PR） |
|---|---|---|
| 踢狗 | `LinuxPlatformHal`：只計數 | `esp_task_wdt_reset()` |
| 重開機原因 | 固定 `POWER_ON` | `esp_reset_reason()` → `WATCHDOG`／`BROWNOUT`／`PANIC`… |
| 迴圈間隔 | 兩拍間隔 > `overrun_s`（1.5 ms）記一次超時 | 同左 |

異常重開（WATCHDOG／BROWNOUT／PANIC）會發 `AbnormalReset` 事件。**要不要進安全態由 FlightMode 決定（FM-20）**，Platform 只回報。

## 遙測／事件／指令
- 遙測（1 Hz）：`resetReason`、`ticks`、`overruns`、`maxLoopDt_s`
- 事件：`Boot(reason)`、`AbnormalReset(reason)`、`LoopOverrun(dt_s)`（最多 20 次，之後只看遙測）
- 指令：`CLEAR_MAX`

## 參數
| 參數 | 預設 | 用途 |
|---|---:|---|
| `loopPeriod_s` | 0.001 | 名目週期 |
| `overrun_s` | 0.0015 | 超過算超時 |

## SilImuFeed（SIL 佔位）
每拍送一筆墊上靜置的 `ImuSample`：比力 `(g·cosθ, 0, g·sinθ)（機體 X 指鼻；θ=0 時 (+g,0,0)）`、角速度 0；開機 `readyDelay_s`（0.1 s）後 `valid=true`。
**只是佔位**：EnvSensing 的 `ImuDriverSim` 目前用真 F´ 編不過（`EnvSensingBus.fpp` 沒進任何 CMake 模組，見 issue），修好後 DsMcu SIL 改接它，這個元件刪除。

## 還沒做（下一個 PR：ESP32）
- `Esp32PlatformHal`（task WDT、reset reason）
- DS 雙核分配（Core 1：1 kHz 鏈；Core 0：其餘）
- 用 `esp_timer` 取代 `Svc.LinuxTimer` 當 1 ms 基準時鐘
- PIL：模擬器經 UART 餵量測

## 檔案
| 檔案 | 內容 |
|---|---|
| `PlatformCore.hpp/.cpp` | 踢狗、間隔、重開原因；`IPlatformHal`、`LinuxPlatformHal` |
| `Platform.fpp/.hpp/.cpp` | F´ 外殼 |
| `SilImuFeed/` | SIL 佔位 IMU |
| `test/PlatformCoreTest.cpp` | 2 站主機測試 |
