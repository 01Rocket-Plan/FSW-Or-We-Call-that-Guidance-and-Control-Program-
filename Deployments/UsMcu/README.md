# UsMcuDeployment

US 飛控 MCU（獨立板）。**不含控制律**：不安裝 Controller、ActuatorMgr、ServoDriver（尾翼），也不負責級間分離（分離時機判斷、SEP_FAIL 與分離供電在 DS，見 gate A9）。

**本版是 Linux SIL**（F´ v4.4.1）。建置與執行：`tools/sil/build_mcu_sil.sh --run 3`（見 `Deployments/DsMcu/README.md`）。

## 速率群組（基準時鐘 1 ms）
| 群組 | 成員 |
|---|---|
| rg1kHz | `flightMode`（role US）→ `platform`（最後，踢狗） |
| rg100Hz | （預留：UsMpu6050、UsDht22） |
| rg1Hz | tlmSend、fileDownlink、systemResources、comQueue、aggregator、cmdDisp、health、commsBufferManager |

## 實例與 Base ID
| 實例 | 元件 | Base ID |
|---|---|---|
| rg1kHz／rg100Hz／rg1Hz | Svc.ActiveRateGroup | 0x0100／0x0200／0x0300 |
| rateGroupDriver、chronoTime、timer、systemResources、comDriver | Svc／Drv | 0x0400–0x0800 |
| platform | Mcu.Platform | 0x1000 |
| flightMode | Mcu.FlightMode（`setDeploymentRole(US)`） | 0x1400 |
| nvFlags | Components.NvFlags（RamNvStore） | 0x6100 |

## SIL 實跑結果（3 s）
```
FlightMode 未填 FM-20（重啟恢復規則待定）；中止指令來源 PIL 接
Platform 開機 reset=POWER_ON
```

## 還沒接的
| 要接的 | 卡在 |
|---|---|
| UsMpu6050、UsDht22 → rg100Hz、sdLogger | 下一輪（先確認能用真 F´ 編譯） |
| gimbalServoDriver（0x3200） | Servo 組模組找不到自己的標頭（issue） |
| 上級開傘驅動 | 沒有元件（FM-IN-7） |
| 相機不併入本部署（見 `Deployments/UsCam`） | — |
