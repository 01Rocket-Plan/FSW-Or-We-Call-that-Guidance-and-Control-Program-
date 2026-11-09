# ActuatorMgr（DS 作動管理 W4）

- 負責：MCU 組（EdwardWang+Claude）
- 部署：**DsMcu only**
- Base ID：`0x1300`
- 速率：rg1kHz，排在 `controller` 之後、`servoDriver`／`sepServoDriver` 之前（Core 1）
- 狀態：v0.1，主機測試 5 站 PASS；F´ v4.4.1 編譯 0 警告

## 一句話
把 Controller 的「尾翼角（rad）」變成舵機看得懂的「脈寬（µs）」，順便擋掉危險的東西；FlightMode 的分離指令也從這裡轉給分離舵機。

```
controller ──FinCmd──►┐                 ┌──ServoCmd──► servoDriver（4×SG90 尾翼）
flightMode ──SepCmd──►│  ActuatorMgr    │
powerMon ───UvState──►│                 └──SepCmd────► sepServoDriver（MG996R 分離）
rg1kHz ─────Sched────►┘
```

## 尾翼路（每 1 ms）

```
fin_rad[i] ─► 速率限制（10.47 rad/s）─► 限幅 ±30° ─► 舵機角 = dir[i]·gear·θ + trim[i]
          ─► µs = 1500 + 舵機角 × 318.31 ─► 硬限 1000–2000 µs
```

| 參數 | 預設 | 意思 |
|---|---:|---|
| `neutral_us`／`pulseMin_us`／`pulseMax_us` | 1500／1000／2000 | 同 `ServoTypes` 的 SG90 常數 |
| `usPerRad` | 318.31 | 1000 µs ÷ 180°（`Sg90Core` 的行程假設，**未實測**） |
| `gear` | 1.0 | 舵機角 ÷ 尾翼角（連桿比） |
| `dir1..4` | +1 | 每通道正反向；**冷測定**，Controller 的符號不動 |
| `trim1..4_rad` | 0 | 機械中位配平 |
| `finMax_rad` | 0.5236（30°） | 跟 Controller 一樣，這裡再擋一次 |
| `rateMax_rads` | 10.47 | SG90 規格書約 0.1 s／60° |
| `finStale_s` | 0.020 | FinCmd 多久沒更新就鎖中位 |

### 鎖中位（任一成立）

| `lockReason` | 條件 |
|---|---|
| `NO_CMD` | 開機後還沒收過 FinCmd |
| `UNDERVOLT` | 欠壓 ≥ `uvLockLevel`（預設 `UV_SHED`） |
| `CMD_STALE` | FinCmd 超過 20 ms 沒更新 |
| `CMD_LOCK` | FinCmd 自己帶 `lockNeutral` |

鎖住時四路 = `neutral + trim`，速率限制狀態歸零，解鎖後從 0 平滑打出去。

> **⚠️ `ServoCmd.lockNeutral` 只在「進入鎖定那一拍」送 true**，之後送中位脈寬、flag=false。
> 原因：`ServoDriver::cmdIn_handler` 每收到一筆 `lockNeutral=true` 就記一次 `LockNeutral` 事件；墊上每 1 ms 送一次會變成每秒 1000 筆事件。輸出的脈寬完全一樣（中位）。若伺服組把事件改成「只在變化時記」，這裡可以改回每拍都送 true。

## 分離路

- 收 FlightMode 的 `SepCmd`：`valid=false` 或 `seq` 不是新的 → 丟棄（`SepDropped`）。
- `sepStagger_s = 0`（預設）：整個 mask 同一拍轉出。
- `sepStagger_s > 0`：照位元順序一顆一顆放，間隔 `sepStagger_s`（SSS §10「三顆 MG996R 錯開 Δ ms」；**Δ 還沒定**）。
- **欠壓不擋分離**（gates：欠壓 → 舵中立、分離不鎖）。
- fire-once、NV 閂鎖由 `SepServoDriver`＋`NvFlags` 負責，這裡不重做。

## 遙測／事件
- 遙測（10 Hz）：`locked`、`lockReason`、`pulse1..4_us`、`clampCount`、`rateLimitCount`、`uvState`、`sepPendingMask`、`sepDropCount`
- 事件：`Locked(reason)`、`Unlocked`、`SepForwarded(mask)`、`SepDropped(seq)`

## 檔案
| 檔案 | 內容 |
|---|---|
| `ActuatorMgrCore.hpp/.cpp` | 全部邏輯；純 C++14 |
| `ActuatorMgr.fpp/.hpp/.cpp` | F´ 外殼 |
| `test/ActuatorMgrCoreTest.cpp` | 5 站主機測試 |
| `run_host_tests.sh`／`run_fpp_check.sh` | 自測 |

## 自測
```
$ ./run_host_tests.sh
ST-ACT-MAP-RATE   第一拍不動、第二拍被速率限制、穩態 0.1 rad → 1532 µs
ST-ACT-LIMIT      限幅 30° → 1667 µs；連桿比放大撞 1000／2000 硬限
ST-ACT-DIR-TRIM   dir=−1 反向；trim 疊加
ST-ACT-LOCK       四種鎖中位原因；UV_WARN 不鎖、UV_SHED 鎖；解鎖平滑起步
ST-ACT-SEP        valid=false 丟、UV_CRITICAL 仍分離、只送一次、錯開 10 ms 三顆依序
ActuatorMgrCore: PASS
```

## 待決（請 Edward 定）
1. `uvLockLevel` 預設 `UV_SHED`（WARN 不鎖）——對嗎？
2. `sepStagger_s` 的 Δ 要多少？目前 0（同時）。
3. `usPerRad` 用的是 180° 行程假設，冷測量完要改。
