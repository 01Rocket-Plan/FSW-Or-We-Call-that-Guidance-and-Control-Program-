# Ov7670Driver（上級相機）

- 負責：相機驅動組（2026-11-10 由 EdwardWang+Claude 依 issue #36／#39 改版）
- 型號 **OV7670**、角色 `US_CAM`（SSS §7；Edward 定案，取代 OV3640）
- 範圍：`Ov7670Core`（狀態機）＋`Ov7670SimDevice`（暫存器／幀模擬）＋ F´ 外殼 `Ov7670Driver`／`Ov7670DriverSim`
- 型別：**全部用 FPP 自動碼**（`../CamTypes/CamTypes.fpp`）。不准再手寫同名 C++ 型別（#36）。
- 影像埠：`imageOut: Components.CamImageOut` → sdLogger（2 KB 分塊）
- Rate group：`sched1Hz`／`sched100Hz`；不接 1 kHz；無 sleep／系統時間／動態配置

## 硬體（OV7670 規格書）
| 項目 | 值 |
|---|---|
| 控制匯流排 | SCCB（I2C 相容），7-bit 位址 **0x21**（寫 0x42／讀 0x43），8-bit 暫存器位址 |
| 識別 | PID 0x0A = 0x76、VER 0x0B = 0x73、MIDH/MIDL 0x1C/0x1D = 0x7F/0xA2 |
| 時鐘 | XCLK 10–48 MHz（典型 24 MHz，由 MCU 提供） |
| 影像 | DVP 8-bit 平行＋VSYNC／HREF／PCLK；**沒有 JPEG**，輸出 RGB565／YUV422 原始像素 |
| 本驅動預設 | 感測器 QCIF 176×144 YUV422（50,688 B）→ 只取 Y＝**灰階 25,344 B**（13 塊 × 2 KB），**1 fps**（Edward 2026-11-14） |
| 電源 | 類比 2.45–3.0 V、IO 1.7–3.0 V（常見模組板上有 LDO，吃 3.3 V） |

腳位由 MCU 組在 ESP32 平台 PR 的全箭腳位表統一分配（DVP 需要 8 條資料＋3 條同步＋XCLK＋SCCB 2 條，共 14 支）。

> ⚠ 資料量：OV7670 沒有壓縮。QVGA RGB565 @10 fps ≈ 1.5 MB/s；VGA（480p）@13 fps ≈ 8 MB/s，ESP32 的 SD
> 寫入跟 RAM 都撐不住。所以定 **144p 灰階 @1 fps ≈ 25 KB/s**。
>
> 灰階怎麼來：YUV422 每像素 2 B（Y0 U0 Y1 V0…），Y 就是亮度＝灰階。驅動在切 2 KB 分塊時直接挑偶數位元組
> （`Ov7670Gray.hpp`），不另外存整幀，不需要額外記憶體。
>
> 1 fps 怎麼來：感測器照自己的速率出幀，`Ov7670Core` 距上次轉存不到 `FRAME_PERIOD_TICKS`（100 個 100 Hz 拍＝1 s）
> 就把這幀丟掉（不轉存、也不算斷訊）。

## 備援：OV3640（有 JPEG）
Edward 2026-11-11：OV7670 留著當主案；另備一份有 JPEG 的 **OV3640** 當備援（相機遙測在紀錄組是 P4 最低優先，
「理論上能傳」即可，不特別為它改 SdLogger）。備援**不合進 main**（同 ICM-20648 的做法），放在分支
`camera/EdwardWang-Claude/ov3640-alt`（＝改 OV7670 之前的 OV3640 版本）。
要啟用時：照本 PR 的做法把它改用 `Components/Camera/CamTypes` 自動碼型別（那個分支還是舊寫法，真 F´ 編不過，見 #36）。

## 初始化（`Ov7670Core.cpp` `kInitTable`）
COM7=0x80 軟重置 → 等一拍（10 ms）→ CLKRC、COM7=0x08（QCIF|YUV）、RGB444 關、COM15=0xC0（YUV 全範圍）、
TSLB=0x04（YUYV 順序）、COM3=0x04（DCW）、COM14=0x11、SCALING_*（PCLK_DELAY 0x52）。
值對照 Linux `drivers/media/i2c/ov7670.c`。

**待冷測核對：** QCIF 的縮放／PCLK 值（COM14、SCALING_DCWCTR、SCALING_PCLK_DIV、SCALING_PCLK_DELAY）和視窗
（HSTART／HSTOP／VSTRT／VSTOP，本表沒寫、用晶片預設）是照 Linux 驅動抄的，還沒上機實拍；冷測時對規格書＋拍一張確認
畫面是 176×144、沒有錯位。YUYV 的 Y 位置（偶數位元組）也在冷測時確認。

## 狀態機（不變）
`OFF →(POWER_ON) INIT →(表寫完) SELFTEST →(ID 對＋收到測試幀) READY`；任一步錯 → `FAULT`。
FAULT 後 1 Hz 每 `FAULT_RECOVER_K` 拍重跑 INIT，最多 `FAULT_RECOVER_R` 次，之後 latch，只有 `RESET` 能清。
超過 `FRAME_MAX_BYTES` → 最後一塊標 `TRUNCATED`、發 `FrameTruncated`、該幀不計 frameCount。

## 驗證（真 F´）
```bash
tools/verify/fprime_verify.sh --ut Components/Camera/CamTypes Components/Camera/Ov7670Driver
```
單元測試 `test/Ov7670SimTest.cpp`（gtest，15 案：含 1 fps 上限、灰階取 Y）。原本的 `test/host_ac/` 假自動碼標頭與 `Ov3640AdapterTest`
已刪除（假的標頭跟假的程式永遠對得上，測不出 #36 這種錯）；F´ 外殼之後改用 F´ 自動產生的 Tester 測。

## SIL
`Deployments/UsCam`：`usCam`（`Ov7670DriverSim`，開機自動上電＋擷取；模擬感測器 10 fps、驅動轉存 1 fps 灰階 25,344 B）→ `sdLogger`（Sim）。
