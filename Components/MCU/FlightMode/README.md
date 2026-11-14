# FlightMode（飛行模式狀態機）

- 邏輯：照 SSS `sss/fsw/flight_mode.py`（2026-11-09 版）1:1 移植，**Edward 2026-11-14 授權 Claude 填**（原 CONTRIBUTING 第 7 條：Bot 不寫轉移邏輯）
- 外殼＋核心：MCU 組（EdwardWang+Claude）
- 部署：DsMcu（`role = DS`）、UsMcu（`role = US`）。**上下級沒有通訊**，各跑各的。
- Base ID：`0x1400`
- 速率：rg1kHz，排在 `estimator` 之後、`controller` 之前
- 依賴：RocketTypes、Power/NvFlags、Wireless/UmbilicalLink（`PadToRocketPort`，架 → 箭）

## 填空狀態

| 編號 | 內容 | SSS 對應 | 狀態 |
|---|---|---|---|
| FM-P | 門檻參數（`FmParams`，數字見下表） | `FlightParams` L103–138 | ✅ |
| FM-01 | 每拍清脈衝 | `tick` L192–194 | ✅ |
| FM-02 | IMU 過期累計 | `tick` L199–202 | ✅ |
| FM-03 | 解保險互鎖 | `tick` L204 | ✅ **repo 差異**：看發射架安檢結論（見下） |
| FM-04 | 地面指令過濾、LEGAL 表 | `tick` L207–223 | ✅ 邏輯；**中止來源 PIL 接**（UplinkAuth） |
| FM-05 | 假分禁傘 | `tick` L225–227 | ✅ |
| FM-06 | D12 自動中止（傾角、角速度、看門狗） | `tick` L229–252 | ✅ |
| FM-07 | PAD_IDLE → ARMED | `tick` L255–259 | ✅ 架進 ARMED／COUNTDOWN |
| FM-08 | ARMED：FIRE、離台、HANGFIRE | `tick` L260–281 | ✅ **repo 差異**：FIRE＝架點火證據（見下） |
| FM-08b | HANGFIRE | `tick` L282–285 | ✅ |
| FM-09 | LIFTOFF → BOOST_PASSIVE | `tick` L286–288 | ✅ |
| FM-10 | BOOST_PASSIVE → BOOST_ACTIVE | `tick` L289–293 | ✅ |
| FM-11 | 燃盡 → COAST | `_maybe_coast` L374–391 | ✅ |
| FM-12 | 分離 | `_maybe_sep` L393–410 | ✅ **進 COAST 後 4 s 分離**（Edward 2026-11-14 暫定；0.8 s 抑制窗保留；`sep_ok` 恆 false） |
| FM-13 | SEPARATED → LS_DESCENT／SEP_FAIL | `tick` L298–307 | ✅ |
| FM-14 | 下級開傘（分離後 3.1 s，accel_high 抑制） | `_maybe_chute_ds` L412–427 | ✅ |
| FM-15 | LS_CHUTE → LS_LANDED | `tick` L310–315 | ✅ |
| FM-16 | SEP_FAIL 側傘 | `tick` L316–317 | ✅ |
| FM-17 | ABORT 後續 | `tick` L318–326 | ✅ |
| FM-18 | 上級（US 部署） | `_tick_us` L478–518 | ✅ **repo 差異**：沒有級間鏈路（見下；問題表 P-1） |
| FM-19 | 組輸出 | `tick` L331–350 | ✅ `sepMask=0x07`（三顆 MG996R）；SEP_FAIL 時 `ChuteCmd.side=true` |
| FM-20 | 重啟恢復（NvFlags phase） | SSS 沒有 | ⏸ 規則待 Edward 定 |

分離開關、著地接觸在軟體裡只是輸入（`DiscreteSample`）。SIL／SSS 用模擬值餵；硬體照這份邏輯與下面的問題表做，
**邏輯不等硬體**。驅動元件還沒有，所以 F´ 部署裡這兩個輸入目前恆 false。

開機會發一次 `LogicIncomplete` 警告（FM-20 未填、中止指令來源 PIL 接）。兩件都完成後刪掉事件。

### 一次飛行的流程（DS）
1. 架 ARMED → 箭 ARMED；架點火（或倒數剩 ≤ 0.15 s）→ 閂 FIRE
2. 加速度 > 5 m/s² 持續 20 ms → LIFTOFF → 下一拍 BOOST_PASSIVE → BOOST_ACTIVE
3. 燃盡（< −5 m/s² 持續 20 ms）→ COAST
4. **COAST + 4 s → 下令分離（三顆舵機）→ SEPARATED**
5. 分離開關斷 → LS_DESCENT；**5 s 內沒斷 → SEP_FAIL**（開側傘、禁上級傘）
6. 分離下令 + 3.1 s → 開傘 → LS_CHUTE
7. 著地接觸，或離台 50 s → LS_LANDED

D12 或已驗證中止 → ABORT：立刻分離＋開傘各一次；之後開關斷 → LS_DESCENT，5 s 沒斷 → SEP_FAIL。

### 上級（US）
開關斷 → US_COAST → 3.2 s 開上級傘 → US_CHUTE → 著地 → US_LANDED。

## 問題表（給硬體）

| 編號 | 問題 | 軟體現況 | 硬體要做的 |
|---|---|---|---|
| P-1 | 上下級不通訊，上級分不出「墊上被碰開（假分）」和「真的分離」 | 上級看到開關斷就當分離，3.2 s 後開傘 → **墊上碰開會在墊上開上級傘** | 擇一：上級分離開關加機械保險（離軌才解除）；或上級加 IMU（UsMpu6050）讓軟體加「已離台」條件 |
| P-2 | 上級開傘計時起點 | 從「上級開關斷」起算（SSS 從下級下令起算），晚一個舵機放開的時間 | 量舵機放開到開關斷的時間；太長就調 `t_chute_us_s` |
| P-3 | 分離開關、著地接觸的電氣定義 | 輸入是 `DiscreteSample.sepSwitchOpen`／`groundContact`（true＝斷開／著地） | 驅動要照這個定義輸出；`hdr.valid=false` 時軟體保留上一筆 |

## repo 差異（和 SSS 不一樣的地方）

### FM-03／07：解保險＝看發射架的安檢結論
SSS 的四條件（seated、點火迴路、READY、地面鏈路）在 repo 由**發射架即時安檢**：PadController 的 `kPadPermitMask`
（箭到位、點火迴路、實體鑰匙、架傾角、導軌、架電、地面鏈路、臍帶鏈路、箭 READY）全綠才會進 ARMED，任一紅燈退回 SAFE。
箭經臍帶 UART（`UmbilicalLink.padStatusOut`）只看架的結論：

| 箭這邊 | 條件 |
|---|---|
| `interlock` | 架狀態夠新（`linkOk`）∧ 架有收到箭（`ROCKET_LINK_OK`）∧ 架在 ARMED／COUNTDOWN／IGNITION／RELEASED／LIFTED_OFF |
| `arm`（SSS ARM 指令） | 同上，且架在 ARMED／COUNTDOWN（架已收到簽名過的 ARM） |

### FM-08：FIRE＝架點火證據（衝突 C-1，gate C15：點火只在架）
箭不在點火迴路裡。ARMED 時看到下列任一就閂 FIRE：
1. 架旗標 `IGNITER_ON`，或架已在 IGNITION／RELEASED／LIFTED_OFF；
2. **架在 COUNTDOWN 且剩 ≤ `t_fire_lead_s`（0.15 s）**——臍帶 10 Hz 一幀，點火那幀可能還沒到臍帶就被拔掉；
   沒有這條，箭會永遠看不到點火、起飛偵測不會開，然後臍帶逾時把箭退回 PAD_IDLE（飛行中！）。`t_fire` 用倒數推回 T=0。

閂了以後（點火後臍帶本來就會拔脫）：
- 架狀態變舊 **不**回 PAD_IDLE（SSS 會；repo 不行）
- 架狀態還新且架回 SAFE（最後一幀後倒數被中止）→ PAD_IDLE、解閂
- 架狀態還新且架回報 HANG_FIRE／HOLD_ABORT／NO_LIFTOFF → HANGFIRE
- 其餘照 SSS：加速度 > 5 m/s² 持續 20 ms → LIFTOFF；`t_fire` 後 1 s 沒離台 → HANGFIRE

### FM-04：中止
SSS 用密碼；repo 用「通過 UplinkAuth（HMAC＋白名單）的 ABORT」＝`FmGroundCmd.abortValid`。外殼目前沒有來源（白名單也還沒有 ABORT opcode），**PIL 時接**。
墊上（離台前）的中止走發射架，不走這裡（SSS 同：只在飛行中受理）。

### FM-18：上級沒有級間鏈路
SSS 的上級從級間鏈路收「下級已下令分離」、下令時刻、禁上級傘。repo 上下級不通訊，這三個都沒有：
- 分離證據＝上級自己的分離開關斷
- 開傘計時起點＝開關斷的那一刻（P-2）
- 禁上級傘：上級收不到下級的 SEP_FAIL／ABORT，也分不出假分（P-1）

## FM-P 參數（`FlightModeCore.hpp` `FmParams`）

| 名稱 | 值 | 用在 |
|---|---:|---|
| `a_liftoff_mps2` | 5.0 | FM-08 |
| `n_accel_s` | 0.020 | FM-08 |
| `t_hang_s` | 1.0 | FM-08 |
| `t_fire_lead_s` | 0.15 | FM-08（repo 加的） |
| `v_active_mps` | 8.0 | FM-10 |
| `t_passive_min_s`／`t_passive_max_s` | 0.10／0.50 | FM-10 |
| `a_burnout_mps2` | −5.0 | FM-11 |
| `n_burnout_s` | 0.020 | FM-11 |
| `t_burnout_min_s`／`t_burnout_max_s` | 0.40／8.0 | FM-11 |
| `t_sep_inhibit_s` | 0.80 | FM-12 |
| `t_sep_backup_s` | **4.0** | FM-12 |
| `t_sep_fail_s` | 5.0 | FM-13、17 |
| `t_chute_ds_s` | 3.1 | FM-14 |
| `t_chute_us_s` | 3.2 | FM-18 |
| `t_land_max_s` | 50.0 | FM-15 |
| `tilt_abort_rad` | 30° | FM-06 |
| `rate_abort_rads` | 100°/s | FM-06 |
| `t_abort_hold_s` | 1.0 | FM-06 |
| `t_abort_min_s` | 0.20 | FM-06 |
| `t_watchdog_s` | 20.0 | FM-06 |
| `t_imu_stale_s` | 0.020 | FM-02 |
| `a_high_mps2` | 5.0 | FM-14 `accel_high`（SSS 夾具定義） |
| `sep_ok` 門檻 | 未定（gates D14） | FM-12（輸入恆 false） |

門檻是 C++ 預設值；要 HIL 現場改再開成 F´ param。SSS 的 `abort_password` **不入庫**。

## 輸入對照

| SSS `Meas` | repo 來源 | 外殼 |
|---|---|---|
| `accel_u_mps2` | `StateOut.accAxial_mps2` | ✅ |
| `v_est_mps` | `|StateOut.vel_enu_mps|` | ✅ |
| `tilt_rad` | `StateOut.tilt_rad` | ✅ |
| `rate_rads` | `hypot(rate_body_rads[1], [2])`（俯仰＋偏航） | ✅ |
| `imu_fresh` | `StateOut.imuValid` | ✅ |
| `accel_high` | `accel_u_mps2 > a_high_mps2`，core 裡算 | ✅ |
| `seated`／`loop_ok`／`ready`／`ground_link` | 架安檢 `padStatusIn`（`padState`、`flags`、`countdown_s`、`linkOk`） | ✅ 埠已加；拓樸待 UmbilicalLink 外殼 |
| `sep_switch_open`／`contact_*` | `discreteIn` → `DiscreteSample` | 埠已接；驅動元件尚無（SIL／SSS 用模擬值） |
| `sep_ok` | — | false |
| 地面指令 | UplinkAuth（PIL） | 全 false |

## 輸出
- `modeOut[6]`：DS 部署廣播 `ds`、US 部署廣播 `us`
- `sepCmdOut`（mask 0x07）＋`sepCommandedOut`（NV 閂鎖）
- `chuteCmdOut`（`stage`、`side`）＋`dsChuteCommandedOut`
- 舵權 `FmOutputs.authority`（SSS `_authority`）：core 已算，Controller 目前用模式自己判斷，還沒接

## 測試
`./run_host_tests.sh` → `test/FlightModeCoreTest.cpp`，21 組（SSS `test_flight_mode.py` 的 CP-* 移植＋repo 差異；開關、著地用模擬值）：

CP-INTERLOCK、CP-HANGFIRE（計時／架回報）、CP-FINAL-COUNT（倒數最後一幀＋臍帶拔脫）、
CP-NOMINAL（ARMED → 4 s 分離 → 3.1 s 開傘 → 落地）、CP-INHIBIT-SEP、CP-BACKUP-SEP、CP-NO-EARLY-SEP、
CP-ABORT-D12、CP-ABORT-TIPOFF、CP-WATCHDOG、CP-CMD-WHITELIST、CP-PAD-ABORT、CP-ILLEGAL-TRANS、CP-CHUTE-DS、
CP-IMU-STALE、CP-PAD-SAFE、CP-SEP-FAIL、CP-FALSE-SEP、CP-ABORT-FOLLOW、CP-LAND-MAX、CP-US。

## 檔案
| 檔案 | 內容 |
|---|---|
| `FlightModeCore.hpp/.cpp` | 狀態機（純 C++14） |
| `FlightMode.fpp/.hpp/.cpp` | F´ 外殼：搬輸入、送輸出、事件、NV 閂鎖；沒有轉移判斷 |
| `test/FlightModeCoreTest.cpp` | 主機測試 |
| `run_host_tests.sh`／`run_fpp_check.sh` | 自測 |
