# 01RocketPlan 飛控軟體（F´）

兩級模型火箭（100 m 構型，Ø80×1100 mm）的 F´ 飛控。同一份元件庫、五份部署：DS／US 各一顆 MCU，相機各自獨立板，發射架一塊板（不上箭）。

> **⚠️ 交差規則（Edward，2026-11-10）：禁止拿 stub 當最終 PR 結果。** 所有最終 PR 都必須用真 F´（`lib/fprime` v4.4.1）建置通過才能上傳：`tools/verify/fprime_verify.sh`。詳見 [`docs/REAL_FPRIME_VERIFICATION.md`](docs/REAL_FPRIME_VERIFICATION.md)。

## 結構
- `RocketTypes/`：共用型別與埠（介面合約 v0.1）
- `Components/<組>/<元件>/`：各組元件
- `Deployments/DsMcu`、`Deployments/UsMcu`、`Deployments/DsCam`、`Deployments/UsCam`、`Deployments/PadMcu`
- `docs/`：協作規則（CONTRIBUTING.md）、慣例（conventions.md）、交差檢驗規則（REAL_FPRIME_VERIFICATION.md）
- `tools/verify/`：真 F´ 建置檢驗腳本

## 部署分工與建置狀態
建置狀態＝用真 F´（`lib/fprime` v4.4.1）在 Linux 建得起來、SIL 跑得動（`bash tools/verify/fprime_verify.sh --ut --run 3`）。ESP32 版（PIL）五份都還沒做。

| 部署 | 板子 | 控制律 | 建置狀態 | 備註 |
|---|---|---|---|---|
| DsMcu | DS 飛控 MCU | 有（Controller／ActuatorMgr／ServoDriver） | ✅ Linux SIL | 1 kHz 即時鏈；級間分離判斷與分離舵機 |
| UsMcu | US 飛控 MCU | **無** | ✅ Linux SIL | 感測、紀錄、上級傘；感測驅動待接 |
| DsCam | ESP32-CAM | — | ✅ Linux SIL | 不併入 DsMcu；Sim 不產生像素 |
| UsCam | OV7670 自有板 | — | ✅ Linux SIL | 不併入 UsMcu |
| PadMcu | 發射架 ESP32 | — | ✅ Linux SIL | 不上箭；發射許可互鎖、點火、夾持（Issue #24）；驗證器與 I/O 驅動未接 → 停在 SAFE |

## 重要程式路徑索引

> 路徑都是 repo 相對連結；行號是寫這份索引時（main `2442ab6`）的位置，改版後可能會偏，以符號名稱為準。
> 「尚未接線」＝元件本身寫好、有測試，但還沒接進 `Deployments/*/Top/topology.fpp`。

### 速率群組（誰跑在幾 Hz）
基準時鐘都是 1 ms（`timer` → `rateGroupDriver`），除數寫在各部署的 `*Topology.cpp` 的 `rateGroupDivisorsSet`；成員順序就是 `topology.fpp` 裡 `connections RateGroups` 的 `RateGroupMemberOut[n]`。

| 部署 | 1 kHz | 100 Hz | 1 Hz | 看哪裡 |
|---|---|---|---|---|
| DsMcu | silImuFeed → estimator → flightMode → controller → actuatorMgr → platform（照資料流排） | baroDriverSim → tlmSend → comQueue → rfDriverSim（無線下行鏈） | F´ 標準服務（CdhCore.tlmSend、fileDownlink、ComCcsds.comQueue、cmdDisp、health…） | [`topology.fpp`](Deployments/DsMcu/Top/topology.fpp#L76)、[`DsMcuTopology.cpp`](Deployments/DsMcu/Top/DsMcuTopology.cpp#L11) |
| UsMcu | flightMode → platform | （空；usMpu6050／usDht22 尚未接線） | F´ 標準服務 | [`topology.fpp`](Deployments/UsMcu/Top/topology.fpp#L62) |
| PadMcu | padController | — | F´ 標準服務 | [`topology.fpp`](Deployments/PadMcu/Top/topology.fpp#L59) |
| DsCam | — | dsCam.sched100Hz、sdLogger | dsCam.sched1Hz、F´ 標準服務 | [`topology.fpp`](Deployments/DsCam/Top/topology.fpp#L59) |
| UsCam | — | usCam.sched100Hz、sdLogger | usCam.sched1Hz、F´ 標準服務 | [`topology.fpp`](Deployments/UsCam/Top/topology.fpp#L59) |

元件 README 裡預定、但尚未接線的速率：ImuDriver 1 kHz、GnssDriver rg100Hz、UmbilicalLink 10 Hz（TlmSend 已在 DsMcu rg100Hz，P1 `rateDiv=10` 約 10 Hz）。DsMcu 尚未接線的連線列在 [`topology.fpp` 的 DsChain 註解](Deployments/DsMcu/Top/topology.fpp#L108)。

### 飛控（MCU）
| 看什麼 | 路徑 | 說明 |
|---|---|---|
| 舵機分配律（4 片尾翼怎麼分） | [`ControllerCore.cpp` `step()`](Components/MCU/Controller/ControllerCore.cpp#L53)（分配在 L89–92） | `fin = (δy, δz, −δy, −δz)`：1/3 片管俯仰、2/4 片管偏航；公式說明在 [`ControllerCore.hpp`](Components/MCU/Controller/ControllerCore.hpp#L13) 開頭 |
| 控制律、增益、限幅 | [`ControllerCore.hpp` `ControllerParams`](Components/MCU/Controller/ControllerCore.hpp#L34)、[`ControllerCore.cpp` `pd()`](Components/MCU/Controller/ControllerCore.cpp#L38) | PD 控制；`kp`、`kd`、`finMax_rad`、`stateStale_s` |
| 發射參考姿態（Status #8） | [`ControllerCore.cpp` `captureLaunchRef()`](Components/MCU/Controller/ControllerCore.cpp#L21)、[`ControllerCore.hpp`](Components/MCU/Controller/ControllerCore.hpp#L20) | 進 BOOST_ACTIVE 第一個合格拍凍結當下姿態當參考，回 PAD_IDLE／ARMED 清掉 |
| 尾翼角 → 舵機脈寬、限速、鎖中位 | [`ActuatorMgrCore.hpp` `ActuatorMgrParams`](Components/MCU/ActuatorMgr/ActuatorMgrCore.hpp#L27)、[`ActuatorMgrCore.cpp` `toPulse()`／`step()`](Components/MCU/ActuatorMgr/ActuatorMgrCore.cpp#L16) | 每通道正反向 `dir`、配平 `trim_rad`、連桿比 `gear`、µs 硬限、`rateMax_rads`、指令過期／欠壓鎖中位；也轉送分離指令 |
| 估態（姿態、高度、傾角） | [`EstimatorCore.cpp` `step()`](Components/MCU/Estimator/EstimatorCore.cpp#L203) | 墊上對準 `align()`、姿態積分、高度 KF；`tilt_rad` 在 L297 |
| 飛行模式狀態機 | [`FlightModeCore.hpp` `FmMode`](Components/MCU/FlightMode/FlightModeCore.hpp#L17)、[`FlightModeCore.cpp` `tick()`](Components/MCU/FlightMode/FlightModeCore.cpp#L97) | PAD_IDLE → ARMED → LIFTOFF → BOOST_PASSIVE／BOOST_ACTIVE → COAST → SEPARATED → 降落傘段；另有 HANGFIRE、SEP_FAIL、ABORT；DS／US 兩種角色（`tickUs()`） |
| 自動中止（傾角 30°、角速率、看門狗） | [`FlightModeCore.cpp` FM-06](Components/MCU/FlightMode/FlightModeCore.cpp#L157)、[`enterAbort()`](Components/MCU/FlightMode/FlightModeCore.cpp#L342)；門檻 [`FlightModeCore.hpp`](Components/MCU/FlightMode/FlightModeCore.hpp#L84) | `tilt_abort_rad`、`rate_abort_rads` 持續 `t_abort_hold_s` → 強制分離＋開傘；離台後 `t_abort_min_s` 內不判 |
| 分離時序 | [`FlightModeCore.hpp`](Components/MCU/FlightMode/FlightModeCore.hpp#L78)、[`maybeSep()`](Components/MCU/FlightMode/FlightModeCore.cpp#L313) | `t_sep_inhibit_s`、`t_sep_backup_s`、`t_sep_fail_s` |
| 踢狗、迴圈監測 | [`Components/MCU/Platform/`](Components/MCU/Platform/README.md) | rg1kHz 最後一個成員；SIL 的 IMU 來源 [`SilImuFeed`](Components/MCU/Platform/SilImuFeed/SilImuFeed.cpp)（佔位用，IMU 靜止，跑不到 BOOST） |
| 非揮發旗標 | [`Components/Power/NvFlags/`](Components/Power/NvFlags/README.md) | 分離／開傘已下令、飛行階段，重開機不重複觸發 |

### 舵機
| 看什麼 | 路徑 | 說明 |
|---|---|---|
| 尾翼舵機驅動（SG90 ×4） | [`Components/Servo/ServoDriver/`](Components/Servo/ServoDriver/README.md) | DsMcu 尚未接線（`actuatorMgr.servoCmdOut` 待接） |
| 級間分離舵機（MG996R ×3） | [`Components/Servo/SepServoDriver/`](Components/Servo/SepServoDriver/README.md) | 尚未接線 |
| PWM 腳位、LEDC 通道 | [`ServoPwmPins.hpp`](Components/Servo/Pwm/ServoPwmPins.hpp) | |
| 接線提案 | [`Components/Servo/INTEGRATION.md`](Components/Servo/INTEGRATION.md) | |

### 發射台（PadMcu）
| 看什麼 | 路徑 | 說明 |
|---|---|---|
| 發射台狀態機 | [`PadControllerCore.hpp`](Components/Wireless/PadController/PadControllerCore.hpp#L10)、[`PadControllerCore.cpp` `tick()`](Components/Wireless/PadController/PadControllerCore.cpp#L230) | SAFE → ARMED → COUNTDOWN → IGNITION → 鬆夾持；任一紅燈／ABORT／DISARM 回 SAFE；一次開機只點火一次 |
| 互鎖（arm 前全綠） | [`computeInterlocks()`](Components/Wireless/PadController/PadControllerCore.cpp#L75)；位元定義 [`PadLinkCodec.hpp`](Components/Wireless/PadLink/PadLinkCodec.hpp#L127) | 到位、導通、鑰匙、傾角、滑軌、架電壓、地面鏈路、臍帶鏈路、箭 ready… |
| 發射架傾角窗（2°～8°，Status #11） | [`PadControllerCore.hpp`](Components/Wireless/PadController/PadControllerCore.hpp#L141)、[`PadController.fpp` 參數](Components/Wireless/PadController/PadController.fpp#L66) | `minTilt_rad`（0x870d）≤ 傾角 ≤ `maxTilt_rad` 才給 `IL_TILT_OK`，含邊界 |
| 發射台全部參數（倒數、推力確認、鬆夾持期限、逾時） | [`PadController.fpp`](Components/Wireless/PadController/PadController.fpp#L43) | `countdown_s`、`thrustConfirm_s`、`releaseDeadline_s`、`groundLinkTimeout_s`、`umbilicalTimeout_s`、`padVmin_V`… |
| 箭↔架、架↔地面三條鏈路與幀格式 | [`Components/Wireless/PadLink/README.md`](Components/Wireless/PadLink/README.md)、[`PadLinkCodec.hpp`](Components/Wireless/PadLink/PadLinkCodec.hpp) | 鬆夾持由架決定，箭用 ready 否決 |
| 地面指令驗證閘 | [`PadCmdGate.hpp`](Components/Wireless/PadLink/PadCmdGate.hpp) | ARM／LAUNCH 等指令要先過驗證；驗證器尚未接，部署停在 SAFE |
| 臍帶（箭端） | [`Components/Wireless/UmbilicalLink/`](Components/Wireless/UmbilicalLink/README.md) | `padStatusOut` → FlightMode 尚未接線 |

### 無線與地面站
| 看什麼 | 路徑 | 說明 |
|---|---|---|
| 下行優先級 P0～P4 | [`ComQueue/README.md` 分層表](Components/Wireless/ComQueue/README.md)、[`ComQueueCore.hpp` `ComPriority`](Components/Wireless/ComQueue/ComQueueCore.hpp#L14)、[`ComQueueCore.cpp`](Components/Wireless/ComQueue/ComQueueCore.cpp) | 同層先進先出、跨層嚴格優先、滿了丟新幀並計數 |
| 遙測取樣 → 封包 | [`Components/Wireless/TlmSend/`](Components/Wireless/TlmSend/README.md) | 每 APID 的 `rateDiv` |
| 空中幀格式 | [`Components/Wireless/Framer/`](Components/Wireless/Framer/README.md) | 同步字、長度、標頭、CRC-32；單幀最大酬載 225 B |
| LoRa 射頻驅動（收發） | [`RfDriverCore.hpp`](Components/Wireless/RfDriver/RfDriverCore.hpp)（接收 `stepRx()`）、[`RfDriverSim/`](Components/Wireless/RfDriverSim/README.md) | TX 照 LoRa 空中時間才算送完（Core 與 Sim 共用 [`LoraTimeOnAir.cpp`](Components/Wireless/RfDriver/LoraTimeOnAir.cpp)；SF7／125 kHz／4:5） |
| 藍牙備用傳輸（BLE，ESP32／STM32WB55） | [`Components/Wireless/BleDriver/`](Components/Wireless/BleDriver/README.md)（[`BleLinkCore.hpp`](Components/Wireless/BleDriver/BleLinkCore.hpp)）、[`BleDriverSim/`](Components/Wireless/BleDriverSim/README.md) | 共用 `RfDriverCore`；切片／重組、掉線丟幀；TX 完成由 BleLinkCore 回報，不用 LoRa 空中時間；未放進部署拓樸 |
| 上行指令驗證、防重放、白名單 | [`Components/Wireless/UplinkAuth/`](Components/Wireless/UplinkAuth/README.md)、[`whitelist/WhitelistGate.hpp`](Components/Wireless/UplinkAuth/whitelist/WhitelistGate.hpp) | ABORT 等上行指令要先過驗證，才會進 FlightMode（`abortValid`） |
| 地面站接收端：缺口／失聯判斷 | [`tools/GDS/LinkLossView.hpp`](tools/GDS/LinkLossView.hpp)、[`README`](tools/GDS/README.md) | 依 `seq` 算缺口；超過 `lossOfSignal_s`（預設 3 s）沒收到合法幀就標 LOS |

注意：DsMcu 的自家無線**下行**鏈 `tlmSend → comQueue → framer → rfDriverSim` 已接在 rg100Hz（[`topology.fpp` 的 WirelessDownlink](Deployments/DsMcu/Top/topology.fpp#L140)）；**上行**（UplinkAuth）與 P0 事件來源**仍未接線**。目前五份 SIL 部署的上下行是 F´ 標準的 `ComCcsds` 子拓樸，經 `Drv.TcpClient`（`comDriver`）連 F´ GDS，見各部署 `topology.fpp` 的 `connections Communications`。程式裡沒有獨立的「地面接收時間窗」排程；跟接收時限有關的是 `lossOfSignal_s`（地面工具）、`groundLinkTimeout_s`／`umbilicalTimeout_s`（發射台）。

### 感測、GNSS、相機、紀錄、電源
| 看什麼 | 路徑 | 說明 |
|---|---|---|
| DS IMU（ICM-20948） | [`Components/EnvSensing/ImuDriver/`](Components/EnvSensing/ImuDriver/README.md) | 輸出已轉成機體系；尚未接線（SIL 用 SilImuFeed） |
| 氣壓計（BME680） | [`Components/EnvSensing/BaroDriver/`](Components/EnvSensing/BaroDriver/README.md) | DsMcu 用 `baroDriverSim` @ 100 Hz |
| US MPU6050、DHT22（只記錄） | [`UsMpu6050/`](Components/EnvSensing/UsMpu6050/README.md)、[`UsDht22/`](Components/EnvSensing/UsDht22/README.md) | 尚未接線 |
| GNSS（TEL0157） | [`Components/Gnss/GnssDriver/`](Components/Gnss/GnssDriver/README.md)、[`GnssDriverSim/`](Components/Gnss/GnssDriverSim/GnssDriverSim.cpp) | 失鎖表在 README；尚未接線 |
| 上級相機 UsCam（OV7670，主相機） | [`Components/Camera/Ov7670Driver/`](Components/Camera/Ov7670Driver/README.md) | UsCam 部署用 `Ov7670DriverSim` |
| 下級相機 DsCam（ESP32-CAM 模組，內建 OV2640） | [`Components/Camera/DsCam/`](Components/Camera/DsCam/README.md)、[`CamTiming.hpp`](Components/Camera/DsCam/CamTiming.hpp) | DsCam 部署用 `Esp32CamSim`；共用型別在 [`CamTypes/`](Components/Camera/CamTypes/README.md) |
| SD 紀錄 | [`Components/Recorder/SdLogger/`](Components/Recorder/SdLogger/README.md) | 紀錄頻道 ID 在 [`RecordChannels.hpp`](Components/Recorder/SdLogger/RecordChannels.hpp) |
| 遙測／事件字典（頻道名、APID） | [`TELEMETRY_EVENT_DICTIONARY.md`](Components/Recorder/TELEMETRY_EVENT_DICTIONARY.md) | |
| 電壓監測、欠壓分級 | [`Components/Power/PowerMonitor/`](Components/Power/PowerMonitor/README.md)、[`UvState/`](Components/Power/PowerMonitor/UvState/README.md) | NOMINAL／UV_WARN／UV_SHED／UV_CRITICAL；接到 ActuatorMgr／FlightMode 尚未接線 |

### 共用合約、慣例、建置
| 看什麼 | 路徑 | 說明 |
|---|---|---|
| 共用型別與埠 | [`RocketTypes/RocketTypes.fpp`](RocketTypes/RocketTypes.fpp) | ImuSample、StateOut、FinCmd、模式等 |
| 座標系、單位、Base ID 分配 | [`docs/conventions.md`](docs/conventions.md) | |
| 協作規則 | [`docs/CONTRIBUTING.md`](docs/CONTRIBUTING.md) | |
| 真 F´ 建置＋UT＋SIL | [`tools/verify/fprime_verify.sh`](tools/verify/fprime_verify.sh) | `bash tools/verify/fprime_verify.sh --ut --run 3`（F´ v4.4.1，`lib/fprime`） |
| 只建 MCU 的 SIL | [`tools/sil/build_mcu_sil.sh`](tools/sil/build_mcu_sil.sh) | `--run 3` 建完各跑 3 秒 |
| 各部署說明 | [`DsMcu`](Deployments/DsMcu/README.md)、[`UsMcu`](Deployments/UsMcu/README.md)、[`PadMcu`](Deployments/PadMcu/README.md)、[`DsCam`](Deployments/DsCam/README.md)、[`UsCam`](Deployments/UsCam/README.md) | 每份 README 有「還沒接的」清單 |

## 慣例
ENU 世界系；機體系照航太業界標準：X 指向鼻錐（滾轉）、Y 右（俯仰）、Z = X×Y（偏航），墊上比力 (+g, 0, 0)（見 docs/conventions.md §2）；Hamilton 純量在前四元數；SI 單位加後綴；時間只來自 F´ 時間服務。

測試順序：SIL → PIL → HIL。自測保持輕量即可。
