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

沿用 SSS `FlightParams` 的增益與限幅（`kp_fin 1.2`、`kd_fin 0.25`、`±30°`）。座標是 conventions §2 的業界標準（2026-11-10）：機體 **X 指鼻錐（滾轉）、Y 右（俯仰）、Z = X×Y（偏航）**——跟 SSS 同一套。

```
u_b = R(q)ᵀ·(0,0,1)            世界 +U 在機體的分量（直立 = (1,0,0)）
e   = x̂ × u_b = (0, −u_z, u_y)   把鼻（機體 X）轉回垂直需要的轉軸；e_y ≈ −俯仰角、e_z ≈ +偏航角
δy  = sat(kp·e_y − kd·ω_y)      俯仰
δz  = sat(kp·e_z − kd·ω_z)      偏航
fin = (δy, δz, −δy, −δz)        = SSS rigid_body.py：TAIL_1/3 管 My、TAIL_2/4 管 Mz
```

- `+δy` 的意思＝**要產生 +Y 力矩的舵量**。實體舵機正反向在 ActuatorMgr 每通道設定，冷測驗證（SSS §5 冷測 2；selftest `ST-ATT-PITCH-SIGN`／`ST-ATT-YAW-SIGN`）；**不改這裡的符號**。
- 不控滾轉：`ω_x` 不進控制律（SSS 也沒有）。

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
- 遙測（10 Hz）：`closedLoop`、`holdReason`、`errPitch_rad`、`errYaw_rad`、`fin1..4_rad`、`satCount`
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
ST-CTL-SIGN-MIX  直立四片 0；俯仰／偏航各歪 0.1 rad → δ = −kp·sin(0.1)；混控 (δy,δz,−δy,−δz)
ST-CTL-DAMP      δy = −kd·ωy、δz = −kd·ωz；滾轉 ωx 5 rad/s 不影響
ST-CTL-SAT       限幅 ±30°
ST-CTL-AUTH      六道擋門各自生效
ControllerCore: PASS
```

## 跟 SSS 一致（2026-11-10 對齊）
SSS 舊版 `gnc/app.py` 的偏航軸有兩個 bug：阻尼用晶片 `gz`（＝滾轉），且剛性項符號反（正回授）。已在 SSS 修掉（新增 selftest `ST-ATT-YAW-HOLD`：修前偏航 8° → 8.84°，修後收斂）。現在 SSS 與本元件是**同一條控制律、同一套機體軸**。

## 待決
- 增益沒經 HIL 調；SSS COVER5 是用同一組數字跑的。
- 舵權表放在 Controller（`modeAllowsClosedLoop`），因為 `ModePort` 只帶模式、不帶舵權。若 Edward 想由 FlightMode 直接下舵權，要改 RocketTypes 加一個 authority 埠。
