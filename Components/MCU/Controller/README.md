# Controller（DS 姿控 W3）

- 負責：MCU 組（EdwardWang+Claude）
- 部署：**DsMcu only**（UsMcu 不含控制律，conventions §4）
- Base ID：`0x1200`
- 速率：rg1kHz，排在 `estimator` 之後、`actuatorMgr` 之前（Core 1）
- 狀態：v0.2（發射參考，Status #8），主機測試 8 站 PASS；F´ v4.4.1 編譯 0 警告

## 一句話
姿態偏離**發射參考**（BOOST 開始那一刻的鼻向）就打尾翼把它扳回來（PD）；**只有 `BOOST_ACTIVE`／`COAST` 才打**，其他時候四片 0 且 `lockNeutral`。

```
estimator ──StateData──►┐
flightMode ─ModePort───►│ Controller ──FinCmdPort──► actuatorMgr
rg1kHz ─────Sched──────►┘
```

## 控制律

沿用 SSS `FlightParams` 的增益與限幅（`kp_fin 1.2`、`kd_fin 0.25`、`±30°`）。座標是 conventions §2 的業界標準（2026-11-10）：機體 **X 指鼻錐（滾轉）、Y 右（俯仰）、Z = X×Y（偏航）**——跟 SSS 同一套。

```
r   = R(q_ref)·x̂               發射參考：BOOST 開始那一拍的鼻向（世界 ENU）；沒捕捉前 = (0,0,1)
u_b = R(q)ᵀ·r                   參考鼻向在機體的分量（姿態 = 參考時 = (1,0,0)）
e   = x̂ × u_b = (0, −u_z, u_y)   把鼻（機體 X）轉回參考需要的轉軸；e_y ≈ −俯仰誤差、e_z ≈ +偏航誤差（相對參考）
δy  = sat(kp·e_y − kd·ω_y)      俯仰
δz  = sat(kp·e_z − kd·ω_z)      偏航
fin = (δy, δz, −δy, −δz)        = SSS rigid_body.py：TAIL_1/3 管 My、TAIL_2/4 管 Mz
```

- `+δy` 的意思＝**要產生 +Y 力矩的舵量**。實體舵機正反向在 ActuatorMgr 每通道設定，冷測驗證（SSS §5 冷測 2；selftest `ST-ATT-PITCH-SIGN`／`ST-ATT-YAW-SIGN`）；**不改這裡的符號**。
- 不控滾轉：`ω_x` 不進控制律（SSS 也沒有）。

## 發射參考（Status #8）

傾斜導軌發射時，目標若寫死成「世界垂直」，一進主動段控制器就會把傾角扳回垂直，抵消導軌傾角的用意。
改成方案 a：**控制目標 = 起飛後第一次閉環那一刻的姿態**（[Status #8](https://github.com/01Rocket-Plan/SSS_Mission_Status/issues/8)）。

| 時機 | 行為 |
|---|---|
| 第一次進 `BOOST_ACTIVE`，且舵權表第 1–4 條都過（有模式、有狀態、狀態新鮮、`imuValid`）的**第一拍** | 凍結當下 `q` 為 `q_ref`，存 `r = R(q_ref)·x̂`；同一拍就對 `r` 閉環，所以捕捉拍舵角 ≈ 0 |
| 進 `BOOST_ACTIVE` 那拍 `imuValid == false`（或狀態太舊） | 不捕捉（那拍本來就不閉環）；在 `BOOST_ACTIVE` 內第一個合格拍才捕捉 |
| 之後的 `BOOST_ACTIVE`／`COAST` | 沿用同一個 `r`，不再重捕 |
| `LIFTOFF`、`BOOST_PASSIVE`、`ABORT` 等其他模式 | 不閉環；參考保持不動 |
| 回到 `PAD_IDLE`／`ARMED` | 清掉參考 → 重新 ARMED 後下一次 `BOOST_ACTIVE` 重新捕捉 |
| 重開機 | core 重建，參考自然清掉；下一個合格的 `BOOST_ACTIVE` 拍重新捕捉 |
| 可閉環但從沒捕捉到參考（目前合法轉換下只有「整段 BOOST 都 `imuValid == false`、到 COAST 才恢復」） | 退回世界垂直（＝舊行為） |

- 不用寫死傾角的參數：參考直接取估態在那一拍的姿態（估態在墊上已用重力對準，實際傾角不一定剛好是導軌標稱值）。
- 直立發射時 `r = (0,0,1)`，控制律跟改之前**逐點一樣**（`ST-CTL-LREF-VERT` 比對 84 點，差 < 1e-6 rad）。
- 只讀模式：外殼 `Controller::toCtlMode()` 把 `FlightModeId` 對映成 core 的 `CtlMode`（`PRE_LAUNCH`／`BOOST_ACTIVE`／`COAST`／`INACTIVE`）。不新增埠、不改 RocketTypes，也不碰 FlightMode 的轉換邏輯。
- 傾角中止條件仍由 FlightMode 以世界垂直為準判斷，本元件不管。
- 什麼時候開始閉環沒有改（仍是舵權表第 5 條：`BOOST_ACTIVE`／`COAST`）。

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
| `Controller.fpp/.hpp/.cpp` | F´ 外殼；`modeAllowsClosedLoop()` 是「哪些模式可閉環」的唯一一行；`toCtlMode()` 把模式對映給 core（發射參考捕捉／清除） |
| `test/ControllerCoreTest.cpp` | 8 站主機測試 |
| `run_host_tests.sh`／`run_fpp_check.sh` | 自測 |

## 自測
```
$ ./run_host_tests.sh
ST-CTL-SIGN-MIX  直立四片 0；俯仰／偏航各歪 0.1 rad → δ = −kp·sin(0.1)；混控 (δy,δz,−δy,−δz)
ST-CTL-DAMP      δy = −kd·ωy、δz = −kd·ωz；滾轉 ωx 5 rad/s 不影響
ST-CTL-SAT       限幅 ±30°
ST-CTL-AUTH      六道擋門各自生效
ST-CTL-LREF-HOLD 傾 5° 在 BOOST 第一拍捕捉、角速度 0 → 四片 ≈ 0；沒參考（COAST 直接進）→ 退回垂直目標
ST-CTL-LREF-PULL 參考 5° 後再俯仰／偏航 ±2° → δ = ∓kp·sin2°，拉回參考而不是垂直
ST-CTL-LREF-VERT 參考 = 垂直 → 與改之前的 PD 逐點一樣（84 點，含阻尼、限幅）
ST-CTL-LREF-ARM  墊上／LIFTOFF／BOOST_PASSIVE 不捕捉；imuValid=false、狀態太舊不捕捉；COAST／ABORT 保持；
                 回 PAD_IDLE／ARMED 清掉、重新 ARMED 後重新捕捉；重開機沒有參考
ControllerCore: PASS
```

## 跟 SSS 一致（2026-11-10 對齊）
SSS 舊版 `gnc/app.py` 的偏航軸有兩個 bug：阻尼用晶片 `gz`（＝滾轉），且剛性項符號反（正回授）。已在 SSS 修掉（新增 selftest `ST-ATT-YAW-HOLD`：修前偏航 8° → 8.84°，修後收斂）。現在 SSS 與本元件是**同一條控制律、同一套機體軸**。

## 待決
- 增益沒經 HIL 調；SSS COVER5 是用同一組數字跑的。
- 舵權表放在 Controller（`modeAllowsClosedLoop`），因為 `ModePort` 只帶模式、不帶舵權。若 Edward 想由 FlightMode 直接下舵權，要改 RocketTypes 加一個 authority 埠。
- 發射參考：可閉環卻從沒捕捉到參考時（整段 BOOST 都 `imuValid == false`）暫定退回世界垂直；若要改成「在 COAST 第一個合格拍捕捉」，等 Edward 決定。
