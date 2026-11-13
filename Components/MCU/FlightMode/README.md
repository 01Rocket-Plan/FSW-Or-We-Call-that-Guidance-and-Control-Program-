# FlightMode（狀態機殼）

- **邏輯負責：Edward 本人**（CONTRIBUTING 第 7 條：Bot 不得撰寫轉移邏輯、觸發、抑制窗或計時器）
- 殼：MCU 組（EdwardWang+Claude）——只有埠、資料結構、空的 `tick()`、輸出接線
- 部署：DsMcu（`role = DS`）、UsMcu（`role = US`）
- Base ID：`0x1400`
- 速率：rg1kHz，排在 `estimator` 之後、`controller` 之前
- 依賴：RocketTypes（#26 `FlightModeId`、#43 `DiscreteData`／`StageLinkPort`／`ChuteCmdPort`，皆已在 main）

## 殼現在會做什麼

| | 行為 |
|---|---|
| DS | 永遠 `PAD_IDLE`，每拍廣播給 6 路 `modeOut` |
| US | 永遠 `US_WAIT` |
| 分離／開傘／NV | **都不會發生** |
| 級間鏈路 | DS 每拍送 `stageLinkOut`（`alive=true`，其餘欄位＝core 的 `bus*`，殼裡全 false／-1） |
| 開機 | 發一次 `ShellOnly` 警告事件，提醒還沒填 |

主機測試 `ST-FM-SHELL-SAFE-*`：餵最壞輸入（全部放行、100 m/s²、傾 3 rad、ARM＋FIRE＋中止）跑 60 s，確認殼不轉移、不分離、不開傘。

## 你要填的只有一個檔：`FlightModeCore.cpp`

外殼（`FlightMode.cpp`）已經把輸入搬好、把輸出送好。你只要在 `FlightModeCore::tick()` 裡填 `[FM-xx]`。要我幫忙時說「**填 FM-12**」就好，不用把整份程式抓下來。

## 填空清單

來源：`SSS_Mission/sss/fsw/flight_mode.py`（2026-11-09 版；行號以這版為準）

| 編號 | 要寫什麼 | SSS 對應 | 需要的輸入 | 狀態 |
|---|---|---|---|---|
| FM-P | 門檻參數（下表） | `FlightParams` L103–138 | — | 數字已有 |
| FM-01 | 每拍開頭清脈衝 | `tick` L192–194 | — | 可填 |
| FM-02 | IMU 過期累計 | `tick` L199–202 | `imu_fresh` ✅ | 可填 |
| FM-03 | 解保險互鎖 | `tick` L204 | `seated` `loop_ok` `ready` `ground_link` ❌ | 卡 FM-IN-1 |
| FM-04 | 地面指令過濾、LEGAL 表 | `tick` L207–223、`LEGAL` | 地面指令 ❌ | 卡 FM-IN-6 |
| FM-05 | 假分禁傘 | `tick` L225–227 | `sep_switch_open` ❌ | 卡 FM-IN-2 |
| FM-06 | D12 自動中止 | `tick` L229–252 | `tilt_rad` `rate_rads` ✅ | 可填 |
| FM-07 | PAD_IDLE → ARMED | `tick` L255–259 | 互鎖、`arm` ❌ | 卡 FM-IN-1/6 |
| FM-08 | ARMED：FIRE、離台、HANGFIRE | `tick` L260–281、`_enter_liftoff` | `accel_u_mps2` ✅；`fire` ⚠️ | 卡衝突 C-1 |
| FM-08b | HANGFIRE | `tick` L282–285 | — | 可填 |
| FM-09 | LIFTOFF → BOOST_PASSIVE | `tick` L286–288 | — | 可填 |
| FM-10 | BOOST_PASSIVE → BOOST_ACTIVE | `tick` L289–293 | `v_est_mps` ✅ | 可填 |
| FM-11 | 燃盡 → COAST | `_maybe_coast` L374–391 | `accel_u_mps2` ✅ | 可填 |
| FM-12 | 分離 | `_maybe_sep` L393–410 | `sep_ok` ❌（門檻未定） | 卡 FM-IN-4 |
| FM-13 | SEPARATED → LS_DESCENT／SEP_FAIL | `tick` L298–307、`_enter_sep_fail` | `sep_switch_open` ❌ | 卡 FM-IN-2 |
| FM-14 | 下級開傘 | `_maybe_chute_ds` L412–427 | `accel_high`（可由 `accel_u_mps2` 算）✅ | 可填（輸出卡 FM-IN-7） |
| FM-15 | LS_CHUTE → LS_LANDED | `tick` L310–315 | `contact_ds` ❌ | 卡 FM-IN-3（可先只用 `t_land_max`） |
| FM-16 | SEP_FAIL 側傘 | `tick` L316–317 | — | 可填 |
| FM-17 | ABORT 後續 | `tick` L318–326、`_enter_abort` | `sep_switch_open` ❌ | 卡 FM-IN-2 |
| FM-18 | 上級（US 部署） | `_tick_us` L478–518、`_advance_us` L455–476 | 級間鏈路 ❌、`contact_us` ❌ | 卡 FM-IN-5 |
| FM-19 | 組輸出（分離 mask、開傘、bus） | `tick` L331–350 | — | 可填；`sepMask` 要你定（硬體 3 顆 MG996R） |
| FM-20 | 重啟恢復（讀 NvFlags phase） | SSS 沒有 | `phaseGetOut` 已接 | 要你定規則（gates：重啟後進安全態） |

**現在就能填、不用等別人的：** FM-01、02、06、08b、09、10、11、14、16、19。

## FM-P 參數（SSS `FlightParams` 的數字，照抄給你參考）

| 名稱 | SSS 值 | 用在 |
|---|---:|---|
| `a_liftoff_mps2` | 5.0 | FM-08 |
| `n_accel_s` | 0.020 | FM-08 |
| `t_hang_s` | 1.0 | FM-08 |
| `v_active_mps` | 8.0 | FM-10 |
| `t_passive_min_s` | 0.10 | FM-10 |
| `t_passive_max_s` | 0.50 | FM-10 |
| `a_burnout_mps2` | −5.0 | FM-11 |
| `n_burnout_s` | 0.020 | FM-11 |
| `t_burnout_min_s` | 0.40 | FM-11 |
| `t_burnout_max_s` | 8.0 | FM-11 |
| `t_sep_inhibit_s` | 0.80 | FM-12 |
| `t_sep_backup_s` | 4.0（2026-11-09 由 10 改） | FM-12 |
| `t_sep_fail_s` | 5.0 | FM-13、17 |
| `t_chute_ds_s` | 3.1 | FM-14 |
| `t_chute_us_s` | 3.2 | FM-18 |
| `t_land_max_s` | 50.0 | FM-15、18 |
| `tilt_abort_rad` | 30° | FM-06 |
| `rate_abort_rads` | 100°/s | FM-06 |
| `t_abort_hold_s` | 1.0 | FM-06 |
| `t_abort_min_s` | 0.20 | FM-06 |
| `t_watchdog_s` | 20.0 | FM-06 |
| `t_imu_stale_s` | 0.020 | FM-02 |
| `sep_ok` 門檻 | **未定**（你記得「< 5 m/s」，未查證；gates D14） | FM-12 |

SSS 的 `abort_password` **不要**放進 repo（值也不要寫在任何檔案或 PR 裡）：CONTRIBUTING 第 6 條禁止入庫密鑰，repo 的中止走 UplinkAuth HMAC（見衝突 C-2）。

## 輸入對照（SSS `Meas` → repo）

| SSS 欄位 | repo 來源 | 外殼已搬？ |
|---|---|---|
| `accel_u_mps2` | `StateOut.accAxial_mps2`（Estimator 已對齊 SSS 定義） | ✅ |
| `v_est_mps` | `|StateOut.vel_enu_mps|` | ✅ |
| `tilt_rad` | `StateOut.tilt_rad` | ✅ |
| `rate_rads` | `hypot(rate_body_rads[1], [2])`（俯仰＋偏航，不含滾轉；SSS fixture 同日修成同一定義） | ✅ |
| `imu_fresh` | `StateOut.imuValid` | ✅ |
| `accel_high` | `accel_u_mps2 > 5`（SSS 夾具的定義）——在 tick 裡算 | 你算 |
| 欠壓 | `UvState` | ✅（`uvState`） |
| `seated` `loop_ok` `ready` `ground_link` | ❌ FM-IN-1 | 填 false |
| `sep_switch_open` | `discreteIn` → `DiscreteSample.sepSwitchOpen`（`hdr.valid=false` 不更新） | ✅（埠已接；驅動元件尚無，未接線前保持 false） |
| `contact_ds` `contact_us` | `discreteIn` → `DiscreteSample.groundContact`（DS 部署寫 `contact_ds`、US 寫 `contact_us`） | ✅（同上） |
| `sep_ok` | ❌ FM-IN-4 | 填 false |
| `link_*`（級間） | `stageLinkIn` → `StageLinkMsg`（`alive`／`sepCmd`／`tSepCmd_s`／`usInhibit`）；逾時由你在 tick 判斷 | ✅（埠已接；級間鏈路元件尚無） |
| `GroundCmd` | ❌ FM-IN-6 | 全 false |

## 缺的介面（repo 目前沒有，不是我能決定的）

| 編號 | 缺什麼 | 誰的 | 建議 |
|---|---|---|---|
| FM-IN-1 | 互鎖四條件 | 無線組（`UmbilicalLink.padStatusOut`，但該模組 CMake 還沒啟用） | 啟用 UmbilicalLink 模組後接 `PadToRocketPort` |
| FM-IN-2 | 分離面微動開關 | 型別＋埠已在 main（#43）、`discreteIn` 已接；**GPIO 驅動元件還沒有** | 新離散輸入驅動 → `DiscreteData` |
| FM-IN-3 | 著地接觸 | 同 FM-IN-2 | 同上，或先只用 `t_land_max` |
| FM-IN-4 | `sep_ok` | **門檻未定** | 你定數字（gates D14） |
| FM-IN-5 | DS↔US 級間鏈路 | 型別＋埠已在 main（#43）、`stageLinkIn`／`stageLinkOut` 已接；**實體鏈路元件還沒有** | 新元件（要首席核） |
| FM-IN-6 | 地面 ARM／中止 指令進 FlightMode | 無線組（UplinkAuth → cmdDisp） | 在 FlightMode.fpp 加 F´ command（你定） |
| FM-IN-7 | **開傘驅動** | 型別＋埠已在 main（#43）、`chuteCmdOut` 已接（`chuteDs`→`stage=DS`、`chuteUs`→`stage=US`，`side` 殼一律 false）；**開傘驅動元件還沒有** | 新驅動（DS 傘、US 傘），fire-once 由驅動＋NvFlags 閂鎖 |

## 衝突（SSS ≠ repo，要你定）

| 編號 | SSS | repo | 影響 |
|---|---|---|---|
| C-1 | 箭上 `cmd.fire` → `motor_on` 點火 | **gate C15：點火只走地面 → 發射架，箭不在點火迴路**（`UmbilicalLink.fpp`） | FM-08 的 FIRE 閂鎖要改成看架的 `PTR_IGNITER_ON`／`PTR_THRUST_CONFIRMED` 旗標？ |
| C-2 | 中止用密碼 `abort_password` | UplinkAuth HMAC＋白名單，密鑰不入庫 | FM-04 改成「收到已驗證的 ABORT 指令」 |
| ~~C-3~~ | ~~機體 X 指向鼻錐~~ | ~~conventions：機體 Z 指向鼻錐~~ | **2026-11-10 已解**：repo 改成業界標準 X 指鼻錐，與 SSS 一致 |

## 檔案
| 檔案 | 內容 |
|---|---|
| `FlightModeCore.hpp/.cpp` | **你要填的地方**：資料結構（照 SSS `ModeState`）＋空 `tick()` |
| `FlightMode.fpp/.hpp/.cpp` | 外殼：搬輸入、送輸出、事件、NV 閂鎖；沒有轉移判斷 |
| `test/FlightModeShellTest.cpp` | 殼安全測試；填完後換成 SSS COVER4 站 |
| `run_host_tests.sh`／`run_fpp_check.sh` | 自測 |
