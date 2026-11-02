module Components {

  @ Edward 2026-11-02 失鎖／重鎖階段（對應 Gnss::LockStage）
  enum GnssLockStage { ACQ, TRACKING, REACQ_FAST, REACQ_MED, REACQ_SLOW, FIX_FAILED }

  @ TEL0157 GNSS 真驅動（UART NMEA：GGA/RMC/VTG/GSA）
  @ 安裝位置：DS／US 皆可；fixOut 只接 tlmSend 與 sdLogger（C6 閉環不依賴 GNSS）。
  @ Base ID：conventions 0x5000；Sim = base+0x80。
  @ schedIn 接 rg100Hz（10 ms）；poll ticks＝POLL_*_MS/10 → 10／100／200；每 poll 一次 UART；不 sleep。
  @ 失鎖表（Edward 2026-11-02）：55s 自 last-good 起算，邊界 15／25／55（LOSS_NO_GGA 計入 FAST）；成功即回 TRACKING。
  @ Gate C6：不自己中止。E4：完全逾時後報錯＝進 FIX_FAILED 並發環境訊息一次。
  passive component GnssDriver {

    @ 速率組 tick（rg100Hz／Svc.RateGroup；內部 tick divider 決定 poll）
    sync input port schedIn: Svc.Sched

    @ GnssFix v0.2 → tlmSend / sdLogger only
    output port fixOut: RocketTypes.GnssData

    @ UART 位元組推入（ByteStream → ring）；僅在 poll 時刻經 IGnssUart::readBytes 讀一次
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

    @ posValid 的 HDOP 上限
    param HDOP_MAX: F32 default 2.5
    @ TRACKING／ACQ 輪詢間隔（ms）
    param POLL_TRACKING_MS: U32 default 100
    @ REACQ_FAST 輪詢間隔（ms）
    param POLL_REACQ_FAST_MS: U32 default 100
    @ REACQ_MED 輪詢間隔（ms）
    param POLL_REACQ_MED_MS: U32 default 1000
    @ REACQ_SLOW／FIX_FAILED 輪詢間隔（ms）
    param POLL_REACQ_SLOW_MS: U32 default 2000
    @ REACQ_FAST 階段時長（秒）
    param REACQ_FAST_S: F32 default 15.0
    @ REACQ_MED 階段時長（秒）
    param REACQ_MED_S: F32 default 10.0
    @ REACQ_SLOW 階段時長（秒）；三段合計 55 s
    param REACQ_SLOW_S: F32 default 30.0
    @ 開機 ACQ 逾時（秒）；逾時一次事件後進 FIX_FAILED（假設 A1）
    param INIT_ACQ_TIMEOUT_S: F32 default 300.0
    @ SELFTEST：收到任意合法 NMEA 的時限（秒）
    param SELFTEST_TIMEOUT_S: F32 default 5.0
    @ 無新 GGA 超過此秒數視為失鎖（含 UART 靜默；不以單次空 poll 判定）
    param LOSS_NO_GGA_S: F32 default 2.0
    @ 連續若干次 poll 收到相同 GGA → 失鎖（STUCK）
    param STUCK_POLLS: U32 default 3

    telemetry HsState: RocketTypes.DrvState
    @ Edward 失鎖表階段
    telemetry LockStage: GnssLockStage
    telemetry FaultCount: U32
    telemetry BadChecksumCount: U32
    telemetry TruncatedCount: U32
    telemetry FixSeq: U32
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

    @ code：Gnss::GnssFault（BAD_CHECKSUM/TRUNCATED/BUS_ERROR/SELFTEST_FAIL/INIT_ACQ_TIMEOUT/FIX_FAILED）
    event Fault(code: U8, count: U32) \
      severity warning high \
      format "GnssDriver fault code={} count={}"
  }

}
