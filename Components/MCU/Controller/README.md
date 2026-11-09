# Controller（DS 姿控 W3）

- 負責：MCU 組（EdwardWang+Claude）
- 部署：**DsMcu only**（UsMcu 不含控制律，conventions §4）
- Base ID：`0x1200`
- 速率：rg1kHz，排在 `estimator` 之後、`actuatorMgr` 之前（Core 1）
- 狀態：v0.1，主機測試 4 站 PASS；F´ v4.4.1 編譯 0 警告

## 一句話
姿態歪了就打尾翼把它扳回來（PD）；**只有 `BOOST_ACTIVE`／`COAST` 才打**，其他時候四片 0 且 `lockNeutral`。

```
estimator ──StateData──►┐
flightMode ─ModePort───►│ Controller ──FinCmdPort──► actuatorMgr
rg1kHz ─────Sched──────►┘
```

## 控制律

沿用 SSS `FlightParams` 的增益與限幅（`kp_fin 1.2`、`kd_fin 0.25`、`±30°`），換成 repo 座標：

```
u_b = R(q)ᵀ·(0,0,1)          世界 +U 在機體的分量（直立 = (0,0,1)）
e   = ẑ × u_b = (−u_y, u_x)   把鼻（機體 Z）轉回垂直需要的轉軸；小角 ≈ 傾角
δx  = sat(kp·e_x − kd·ω_x)
δy  = sat(kp·e_y − kd·ω_y)
fin = (δx, δy, −δx, −δy)      同 SSS 混控：TAIL_1/3 一對管 X，TAIL_2/4 一對管 Y
```

- `+δx` 的意思＝**要產生 +X 力矩的舵量**。實體舵機正反向在 ActuatorMgr 每通道設定，冷測驗證（SSS §5 冷測 2）；**不改這裡的符號**。
- 不控滾轉：`ω_z` 不進控制律（SSS 也沒有）。

## 舵權（誰准閉環）

照 SSS `flight_mode.py` 的 `_authority` 寫，先到先擋：

| 順序 | 條件 | 結果（`holdReason`） |
|---|---|---|
| 1 | 還沒收過 ModePort | 鎖中位（`NO_MODE`） |
| 2 | 還沒收過 StateOut | 鎖中位（`NO_STATE`） |
| 3 | StateOut 比 `stateStale_s`（20 ms）舊 | 鎖中位（`STATE_STALE`） |
| 4 | `imuValid == false`（IMU 過期或未對準） | 鎖中位（`IMU_INVALID`） |
| 5 | 模式不是 `BOOST_ACTIVE`／`COAST` | 鎖中位（`MODE_INACTIVE`） |
| 6 | `demo50mFixed == true`（A9 Demo-1-1） | 鎖中位（`DEMO_FIXED`） |
| — | 全過 | **閉環**（`NONE`） |

`ABORT`、分離後、`SEP_FAIL`、被動助推、墊上、`HANGFIRE` 全落在第 5 條。SSS 的 `CENTER` 與 `LOCKED_CENTER` 在 FinCmd 上都是「四片 0＋`lockNeutral`」，這裡不另分。

## 參數（prmDb）

| 參數 | 預設 | 來源 |
|---|---:|---|
| `kp` | 1.2 | SSS `kp_fin` |
| `kd` | 0.25 | SSS `kd_fin` |
| `finMax_rad` | 0.5236（30°） | SSS `fin_max_rad` |
| `stateStale_s` | 0.020 | SSS `t_imu_stale_s` |
| `demo50mFixed` | false | SSS `demo_50m_fixed` |

## 遙測／事件
- 遙測（10 Hz）：`closedLoop`、`holdReason`、`errX_rad`、`errY_rad`、`fin1..4_rad`、`satCount`
- 事件：`ClosedLoopOn`、`ClosedLoopOff(reason)`

## 檔案
| 檔案 | 內容 |
|---|---|
| `ControllerCore.hpp/.cpp` | PD＋舵權；純 C++14 |
| `Controller.fpp/.hpp/.cpp` | F´ 外殼；`modeAllowsClosedLoop()` 是「哪些模式可閉環」的唯一一行 |
| `test/ControllerCoreTest.cpp` | 4 站主機測試 |
| `run_host_tests.sh`／`run_fpp_check.sh` | 自測 |

## 自測
```
$ ./run_host_tests.sh
ST-CTL-SIGN-MIX  +X 歪 0.1 rad → δx = −kp·sin(0.1)；混控 (δx,δy,−δx,−δy)
ST-CTL-DAMP      δ = −kd·ω；滾轉 5 rad/s 不影響
ST-CTL-SAT       限幅 ±30°
ST-CTL-AUTH      六道擋門各自生效
ControllerCore: PASS
```

## 跟 SSS 的差異（刻意的）
SSS `gnc/app.py` 的偏航阻尼用 `gz`，但照 SSS 自己的 `body_to_imu`，晶片 `gz` 是**滾轉軸**（沿鼻），所以那一項其實在用滾轉角速度阻尼偏航。這裡用正確的橫向軸 `ω_x`／`ω_y`。SSS 那份是佔位版（註解寫「估態以後放這裡」），這裡沒照抄。

## 待決
- 增益沒經 HIL 調；SSS COVER5 是用同一組數字跑的。
- 舵權表放在 Controller（`modeAllowsClosedLoop`），因為 `ModePort` 只帶模式、不帶舵權。若 Edward 想由 FlightMode 直接下舵權，要改 RocketTypes 加一個 authority 埠。
