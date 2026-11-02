module Components {

  @ Edward 2026-11-02 失鎖／重鎖階段（對應 Gnss::LockStage）
  @ ACQ：開機搜星；逾時→FIX_FAILED（非 FAULT，因屬環境／天線，C6 可恢復）
  @ TRACKING：已鎖定；失鎖→REACQ_FAST
  @ REACQ_FAST／MED／SLOW：自 last-good 起 0–15／15–25／25–55 s；成功即回 TRACKING
  @ FIX_FAILED：≥55 s 或 ACQ 逾時；發環境原文一次；仍以 SLOW 間隔輪詢，可回 TRACKING
  enum GnssLockStage { ACQ, TRACKING, REACQ_FAST, REACQ_MED, REACQ_SLOW, FIX_FAILED }

  @ TEL0157 GNSS 真驅動（UART NMEA：GGA/RMC/VTG/GSA）
  @ 安裝位置：DS／US 皆可；fixOut 只接 tlmSend 與 sdLogger（C6 閉環不依賴 GNSS）。
  @ Base ID：conventions 0x5000；Sim = base+0x80。
  @
  @ 整數 tick 分頻（為何不用 float 秒差）：
  @   schedIn＝rg100Hz（10 ms）。periodTicks = POLL_*_MS / 10 → 預設 10／100／200。
  @   未滿 period 的拍直接 return，不 sleep、每 poll 最多一次 UART read。
  @   float 秒差在 0.01 累加下會漂成 ~110 ms，導致 T12 FAST poll 偏少。
  @
  @ 失鎖表時鐘：55 s 自「最後合格 GGA」起算；邊界 15／25／55。
  @   LOSS_NO_GGA 偵測耗時計入 FAST（避免 2+15=17）。
  @ Gate C6：不自己中止。E4：完全逾時後報錯＝FIX_FAILED＋環境訊息一次。
  @
  @ emit／seq：新 GGA 立刻發；重鎖／靜默最多 1 Hz 降級（posValid=false）；
  @   每次實際輸出 seq++（有 emit 才遞增）。
  @
  @ 故障分流：BUS／SELFTEST → DrvState::FAULT（硬）；ACQ 逾時／55 s → FIX_FAILED（軟，仍 READY）。
  passive component GnssDriver {

    @ 速率組 tick（rg100Hz／Svc.RateGroup；內部整數 tick 分頻決定是否 poll）
    sync input port schedIn: Svc.Sched

    @ GnssFix v0.2 → tlmSend / sdLogger only（閉環不接此埠）
    output port fixOut: RocketTypes.GnssData

    @ UART 位元組推入 ring；僅在 poll 時刻經 IGnssUart::readBytes 讀一次（規則）
    sync input port uartRecv: Drv.ByteStreamData
    @ 歸還 uartRecv 緩衝區所有權
    output port uartRecvReturn: Fw.BufferSend

    @ F´ 時間服務
    time get port timeCaller
    @ 參數資料庫讀取（prmDb）
    param get port prmGetOut
    @ 參數資料庫寫入（prmDb）
    param set port prmSetOut
    @ 遙測輸出（tlmSend）
    telemetry port tlmOut
    @ 事件日誌
    event port logOut
    @ 文字事件
    text event port logTextOut
    @ 指令分發器（Svc.CommandDispatcher / cmdDisp）
    command recv port cmdIn
    @ 指令分發器 — 註冊
    command reg port cmdRegOut
    @ 指令分發器 — 回應
    command resp port cmdResponseOut

    @ posValid 的 HDOP 上限（無單位比值）。超過則不可標合格 3D，並可視為失鎖。
    param HDOP_MAX: F32 default 2.5
    @ TRACKING／ACQ 輪詢間隔（ms）。÷ rgPeriod(10) → 10 ticks。緊密追蹤／搜星。
    param POLL_TRACKING_MS: U32 default 100
    @ REACQ_FAST 輪詢間隔（ms）。剛失鎖仍高頻，利於快速重鎖。→ 10 ticks。
    param POLL_REACQ_FAST_MS: U32 default 100
    @ REACQ_MED 輪詢間隔（ms）。中段省匯流排／CPU。→ 100 ticks。
    param POLL_REACQ_MED_MS: U32 default 1000
    @ REACQ_SLOW／FIX_FAILED 輪詢間隔（ms）。末段最低負荷續等。→ 200 ticks。
    @ 緩衝：2 s @9600 ≈1920 B → ring／kPollReadMax=2048。
    param POLL_REACQ_SLOW_MS: U32 default 2000
    @ REACQ_FAST 配額（秒）：自 last-good 起算至本邊界（預設 15）。
    param REACQ_FAST_S: F32 default 15.0
    @ REACQ_MED 配額（秒）：接在 FAST 後（預設 10 → 邊界 25）。
    param REACQ_MED_S: F32 default 10.0
    @ REACQ_SLOW 配額（秒）：接在 MED 後（預設 30 → 邊界 55＝三段合計）。
    param REACQ_SLOW_S: F32 default 30.0
    @ 開機 ACQ 逾時（秒）。逾時一次事件後進 FIX_FAILED（A1），非 FAULT（環境可恢復）。
    param INIT_ACQ_TIMEOUT_S: F32 default 300.0
    @ SELFTEST：須在此時限（秒）內收到任意合法 NMEA，否則 FAULT（帶起／接線失敗）。
    param SELFTEST_TIMEOUT_S: F32 default 5.0
    @ 無新 GGA 超過此秒數視為失鎖（含 UART 靜默）。須 > NMEA 週期；不以單次空 poll 判定。
    @ 此偵測時間計入 REACQ_FAST（outageAge 用 last-good）。
    param LOSS_NO_GGA_S: F32 default 2.0
    @ 連續若干次 poll 收到相同 GGA 內容 → STUCK 失鎖（接收機卡住）。
    param STUCK_POLLS: U32 default 3

    telemetry HsState: RocketTypes.DrvState
    @ Edward 失鎖表階段（遙測）
    telemetry LockStage: GnssLockStage
    telemetry FaultCount: U32
    telemetry BadChecksumCount: U32
    telemetry TruncatedCount: U32
    @ 最近一次 emit 的 seq（每次輸出遞增）
    telemetry FixSeq: U32
    @ 自 last-good（或 last GGA）起的資料年齡（秒）
    telemetry DataAge: F32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "GnssDriver {} -> {}"

    event LockStageChanged(fromStage: GnssLockStage, toStage: GnssLockStage) \
      severity activity high \
      format "GnssDriver lock {} -> {}"

    @ Edward 2026-11-02：進入 FIX_FAILED 時只發一次（原文勿改）
    event FixFailedOpenSky \
      severity warning high \
      format "請檢察環境開放條件以利天線運作"

    @ code：Gnss::GnssFault
    @   硬→FAULT：BUS_ERROR、SELFTEST_FAIL
    @   軟（仍 READY）：INIT_ACQ_TIMEOUT、FIX_FAILED；另有 BAD_CHECKSUM／TRUNCATED 計數事件
    event Fault(code: U8, count: U32) \
      severity warning high \
      format "GnssDriver fault code={} count={}"
  }

}
