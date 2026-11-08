# ComQueue

- 負責：無線組（子Bot1）
- Base ID：`0x8200`
- 範圍：P0..P4 下行優先佇列；`buffOut → framer`；`comStatusIn ← rfDriver`
- **passive**＋**guarded** `comIn[5]`（多 rate group 入列，元件 mutex）；`run`＝sync，僅 Core 0 rg100Hz
- 每 `run` 最多 `maxPktsPerRun`（預設 4，≥1）幀；不一次排空
- Viktor：若 rg 預算爆掉再改 active

## 行為摘要
- 同層 FIFO；跨層 P0→P4 嚴格優先
- 全層滿丟最新（拒收並計數）；P0 放關鍵事件，先發生的不能被後來的擠掉
- 只留最新（`setLatestOnly(層, apid)`）：該 apid 在該層最多排一幀，新幀原位覆蓋未送出的舊幀，計入 `coalesceCount`、不算丟包。P0 的 GNSS fix 用這個：送出去的一定是最新座標，也不會塞滿 P0 擠掉事件；P0 已被事件塞滿且沒有排隊中的 fix 時，新 fix 照常被拒（事件優先）
- `EVT_QUEUE_FULL_DROP` ≤1 次／100 tick，附累計數
- `comStatusIn` 忙碌時不推進
- Core 無 sleep／無鎖；執行緒安全靠 F´ guarded mutex（host 測試用 `std::mutex` 模擬）

## 分層表（誰進哪一層）

Edward 2026-11-08 定案、2026-11-09 改 GNSS fix 進 P0（PR #17 審查）。頻寬飽和時 GPS 與關鍵事件最先送，其次 P1 導航，再來健康／診斷。
入列端（TlmSend、各事件來源）在拓樸接線時照此表接到 `comIn[層號]`；頻道名見
`Components/Recorder/TELEMETRY_EVENT_DICTIONARY.md`。

| 層 | 內容 | 來源元件 |
|---|---|---|
| P0 | 關鍵事件：模式轉移、指令被拒（CmdRejected）、ABORT 回覆、SEP 分離、開傘指令、IMU 失效、低壓卸載；**GNSS fix（位置），只留最新** | FlightMode、UplinkAuth、SepServoDriver、ImuDriver、UvStateComp、GnssDriver |
| P1 | 導航：Estimator 姿態四元數／傾角／速度、IMU | Estimator、ImuDriver |
| P2 | 健康：flightMode 週期回報、電源三軌電壓、舵面指令角 | FlightMode、PowerMonitor、ActuatorMgr |
| P3 | 診斷：各元件 handshake、佇列 HWM／丟包數、GNSS data age | 各驅動、各佇列擁有者、GnssDriver |
| P4 | 可犧牲：相機縮圖／分塊（若下傳）、除錯文字事件 | DsCam／UsCam、各元件 |

GNSS fix 的 apid 尚未定義；拓樸整合時定好後，在 F´ 殼初始化呼叫 `setLatestOnly(P0, <GNSS fix apid>)`。

嚴格優先代表 P0／P1 持續滿載時 P3／P4 會餓死，這是預期行為；`QueueHwmP*`、`DropCountTotal` 可在地面看出哪層在丟。
