# DsMcuDeployment

DS 飛控 MCU（獨立板），含控制律。**本版是 Linux SIL**（F´ v4.4.1）；ESP32 版下一個 PR，連線圖相同、只換時鐘與 HAL。

## 怎麼建、怎麼跑
```bash
git submodule update --init lib/fprime          # F´ v4.4.1
pip install fprime-tools==4.4.0 fprime-fpp==3.4.0
tools/sil/build_mcu_sil.sh --run 3              # 建 DsMcu＋UsMcu，各跑 3 秒印事件
```
> 為什麼不在根目錄 `cmake`：F´ v4 configure 時會對整個專案跑 `fpp-depend`，目前其他組有 fpp 錯誤，根專案 configure 不起來（見 issue）。修好後根目錄就能直接建。

連 GDS：`Deployments_DsMcu -a 127.0.0.1 -p 50000`（先開 `fprime-gds`）。

## 速率群組（基準時鐘 1 ms）
| 群組 | 成員（依序） |
|---|---|
| **rg1kHz** | `silImuFeed` → `estimator` → `flightMode` → `controller` → `actuatorMgr` → `platform`（最後，踢狗） |
| rg100Hz | `baroDriverSim` |
| rg1Hz | tlmSend、fileDownlink、systemResources、comQueue、aggregator、cmdDisp、health、commsBufferManager |

1 kHz 的順序就是資料流：IMU 先出樣本 → 估態 → 模式（用這拍的估態）→ 控制（用這拍的模式）→ 作動 → 踢狗。

## 資料流
```
silImuFeed ─ImuData──►┐
baroDriverSim ─Baro──►│ estimator ─State[0]─► controller ─FinCmd─► actuatorMgr ─ServoCmd─► （待接 servoDriver）
                      │           ─State[1]─► flightMode ─Mode[0]─► estimator
                      │                                  ─Mode[1]─► controller
                      │                                  ─SepCmd──► actuatorMgr ─SepCmd──► （待接 sepServoDriver）
                      │                                  ─閂鎖／phase─► nvFlags（RAM 後端）
```

## 實例與 Base ID
| 實例 | 元件 | Base ID |
|---|---|---|
| rg1kHz／rg100Hz／rg1Hz | Svc.ActiveRateGroup | 0x0100／0x0200／0x0300 |
| rateGroupDriver、chronoTime、timer、systemResources、comDriver | Svc／Drv | 0x0400–0x0800 |
| platform | Mcu.Platform | 0x1000 |
| silImuFeed | Mcu.SilImuFeed（SIL 佔位） | 0x1080 |
| estimator | Mcu.Estimator | 0x1100 |
| controller | Mcu.Controller | 0x1200 |
| actuatorMgr | Mcu.ActuatorMgr | 0x1300 |
| flightMode | Mcu.FlightMode（role DS） | 0x1400 |
| baroDriverSim | Components.BaroDriverSim | 0x4180 |
| nvFlags | Components.NvFlags（RamNvStore） | 0x6100 |
| CdhCore／ComCcsds／FileHandling | F´ 子拓樸 | 0x01000000／0x02000000／0x05000000 |

## SIL 實跑結果（3 s）
```
BaroDriverSim OFF → INIT → SELFTEST → READY
FlightMode 未填 FM-20（重啟恢復規則待定）；中止指令來源 PIL 接
ActuatorMgr 尾翼鎖中位 reason=CMD_LOCK        ← Controller 在非主動模式送 lockNeutral，正確
Platform 開機 reset=POWER_ON
Estimator 墊上對準完成 tilt_rad=0.000000       ← 約開機後 0.6 s（0.1 s IMU 自檢 + 0.5 s 對準）
```

## 還沒接的（等其他組模組能用真 F´ 編譯）
| 要接的 | 卡在 |
|---|---|
| `actuatorMgr.servoCmdOut → servoDriver(Sim)` | Servo 組模組找不到自己的標頭（issue） |
| `actuatorMgr.sepCmdOut → sepServoDriver(Sim)` | 同上 |
| `imuDriver(Sim) → estimator.imuIn`（取代 silImuFeed） | `EnvSensingBus.fpp` 沒進 CMake（issue） |
| `powerMonitor／UvState → actuatorMgr.uvIn、flightMode.uvIn` | PowerMonitor 指令 handler 簽名不合 v4.4.1（issue） |
| `flightMode.modeOut[2..] → umbilicalLink、sdLogger、tlmSend` | 無線組模組 CMake 未啟用；SdLogger 可以，下一輪接 |
| `gnssDriver`、`sdLogger`、無線鏈 | 同上 |
