# Estimator（DS 估態 W1）

- 負責：MCU 組（EdwardWang+Claude）
- 部署：**DsMcu only**（UsMcu 的 MPU6050 不進迴圈，SSS §7）
- Base ID：`0x1100`（docs/conventions.md §5）
- 速率：rg1kHz，排在 `imuDriver` 之後、`controller` 之前（Core 1）
- 狀態：v0.1，主機測試 7 站 PASS；F´ v4.4.1 編譯 0 警告

## 一句話
吃 IMU（機體系）和氣壓，吐 `StateOut`：**姿態四元數、角速度、傾角、垂直高度／速度、垂直加速度**。

```
imuDriver ──ImuData──►┐
baroDriver ─BaroData─►│ Estimator ──StateData[0..3]──► controller / flightMode / sdLogger / tlmSend
flightMode ─ModePort─►┘      ▲
rg1kHz ─────Sched────────────┘
```

## 座標（conventions §2，2026-11-10 業界標準）
世界 ENU；機體 **X 指鼻錐（滾轉）、Y 右（俯仰）、Z = X×Y（偏航）**；墊上比力 `(+g, 0, 0)`。
輸入的 `ImuSample` 必須已經是機體系（ImuDriver 用 prmDb 的 DCM 轉）。

## 怎麼算的（白話）

| 東西 | 方法 | 什麼時候 |
|---|---|---|
| 對準 | 墊上平均 `padAlign_s`（0.5 s）的加速度 → 「上」在哪；同時平均陀螺 → 零偏 | 只在墊上（`PAD_IDLE`／`ARMED`） |
| 姿態 | 四元數陀螺積分（每拍一次，指數映射） | 對準後一直做 |
| 水平修正 | Mahony 比例修正：加速度量到的「上」拉回估計的「上」 | **只在墊上**且靜止（\|f\|≈g） |
| 陀螺零偏 | 墊上靜止時低通追蹤（τ=2 s） | 離台後凍結 |
| 垂直高度／速度 | 2 態線性 KF `[h, v]`：加速度預測（1 kHz）＋氣壓更新（~100 Hz）＋墊上零速更新 | 對準後 |
| 水平速度 E/N | 純積分，**無修正**（GNSS 不進估態，conventions §3） | 對準後；墊上歸零 |

離台後**不再用加速度修水平**：推力、阻力會把「上」騙歪。100 m 飛行只有約 10 s，純陀螺積分的漂移在零偏扣掉後夠用。

## 輸出欄位的意思（StateOut v1.0）

| 欄位 | 意思 | 注意 |
|---|---|---|
| `q_wxyz` | 機體→世界（ENU），Hamilton，w 先 | 對準前是單位四元數 |
| `rate_body_rads` | 機體角速度（已扣零偏） | |
| `tilt_rad` | 鼻（機體 X）離世界 +U 的角 | D12 中止用 |
| `vel_enu_mps` | E/N 純積分；U 是 KF | |
| `velValid` | `imuValid` 且氣壓 ≤ `baroMaxAge_s` | |
| `alt_m` | **相對發射台**（墊上氣壓基準） | 不是海拔 |
| `baroAge_s` | 距上一筆有效氣壓 | 從沒收過＝1e6 |
| `accAxial_mps2` | **世界 U 向運動學加速度（已扣 g）** | ⚠️ 名稱沿用 StateOut v1.0，值對齊 SSS `accel_u_mps2`（離台 >5、燃盡 <−5 那個） |
| `imuValid` | IMU 新鮮（≤ `imuStale_s`＝20 ms）**且已對準** | Controller 據此鎖中位 |
| `gyroSat` | 直通 IMU 當拍 | |

## 參數（prmDb）

| 參數 | 預設 | 單位 | 用途 |
|---|---:|---|---|
| `g_mps2` | 9.80665 | m/s² | 重力 |
| `padAlign_s` | 0.5 | s | 對準平均時間 |
| `accGate_mps2` | 0.5 | m/s² | \|\|f\|−g\| 小於此值算靜止 |
| `rateGate_rads` | 0.05 | rad/s | 零偏追蹤的靜止門檻 |
| `kLevel_rads` | 0.5 | 1/s | 墊上水平修正增益 |
| `gyroBiasTau_s` | 2.0 | s | 零偏低通 |
| `qAcc_m2ps4` | 0.25 | (m/s²)² | KF 程序雜訊 |
| `rBaro_m2` | 0.25 | m² | 氣壓量測方差（σ 0.5 m） |
| `baroGate_m` | 10 | m | 氣壓新息超過就丟 |
| `rZupt_m2ps2` | 1e-4 | (m/s)² | 墊上零速更新 |
| `altRefTau_s` | 5 | s | 墊上氣壓基準追蹤 |
| `imuStale_s` | 0.020 | s | 對齊 SSS `t_imu_stale_s` |
| `baroMaxAge_s` | 1.0 | s | 氣壓太舊 → velValid=false |
| `dtMax_s` | 0.005 | s | 單步積分上限 |

數字都是**起始值**，不是實測調出來的；HIL 再調。

## 遙測／事件
- 遙測（10 Hz）：`aligned`、`onPad`、`tilt_rad`、`alt_m`、`velU_mps`、`accAxial_mps2`、`baroAge_s`、`gyroBias{X,Y,Z}_rads`、`baroRejectCount`、`imuStaleCount`
- 事件：`Aligned`、`ImuStale`、`ImuRecovered`、`LeftPad`

## 檔案
| 檔案 | 內容 |
|---|---|
| `EstimatorCore.hpp/.cpp` | 全部數學；純 C++14，無 F´ |
| `Estimator.fpp/.hpp/.cpp` | F´ 外殼：搬資料、取時間、扇出、遙測 |
| `test/EstimatorCoreTest.cpp` | 7 站主機測試 |
| `run_host_tests.sh` | 跑主機測試（自動找編譯器） |
| `run_fpp_check.sh` | fpp-check（用 EnvSensing 的 stub） |

## 自測
```
$ ./run_host_tests.sh
ST-EST-PAD-ALIGN / TILT-ALIGN / PAD-LEVEL / GYRO / VERT-KF / IMU-STALE / BARO-GATE
EstimatorCore: PASS
$ ./run_fpp_check.sh
fpp-check OK
```

## 已知限制／待決
1. **E/N 速度無修正**：只供紀錄，不進任何轉移條件。
2. **墊上判斷靠 ModePort**：只認 `PAD_IDLE`／`ARMED`；`HANGFIRE` 合併 RocketTypes PR #26 後可再加。
3. **跨核**：輸入埠都是 `guarded`（互斥鎖）。若 Platform 把氣壓放 Core 0，鎖會跨核；PIL 量最壞執行時間時要看這把鎖。
4. 參數未經 HIL 調整。
