module Components {

  @ TEL0157 GNSS 真驅動（UART NMEA：GGA/RMC/VTG/GSA）
  @ 安裝位置：DS／US 皆可（由拓撲決定）；fixOut 只接 tlmSend 與 sdLogger，不進 estimator。
  @ Base ID：conventions.md 為 0x5000；先前暫定 0x0A00，拓撲請以 conventions 為準。Sim = base+0x80。
  @ schedIn 接 rg1Hz；不 sleep、不讀系統時間；每 tick 最多一次 UART 讀取。
  @ Gate C6：飛行中失鎖／UART 斷線不自己中止；降級輸出（posValid=false、dataAge↑、FIX_LOST／NO_DATA edge）；
  @          預設 NODATA_FAULT_TICKS=0（永不因靜默進 FAULT）；恢復後 FIX_RECOVERED。
  passive component GnssDriver {

    @ 速率組 tick（rg1Hz／Svc.RateGroup）
    sync input port schedIn: Svc.Sched

    @ GnssFix v0.2 → tlmSend / sdLogger only（不進 estimator；C6 閉環不依賴 GNSS）
    output port fixOut: RocketTypes.GnssData

    @ UART 位元組推入（ByteStream 驅動 → 本元件 ring buffer）；核心經 IGnssUart 每 tick 讀一次
    sync input port uartRecv: Drv.ByteStreamData
    @ 歸還 uartRecv 緩衝區所有權（Fw.BufferSend）
    output port uartRecvReturn: Fw.BufferSend

    @ F´ 時間服務（核心不讀系統時間）
    time get port timeCaller
    @ 參數資料庫讀取（prmDb / Svc.PrmDb）
    param get port prmGetOut
    @ 參數資料庫寫入（prmDb / Svc.PrmDb）
    param set port prmSetOut
    @ 遙測輸出（tlmSend / Svc.TlmChan）
    telemetry port tlmOut
    @ 事件日誌（Svc.EventManager）
    event port logOut
    @ 文字事件（Svc.PassiveConsoleTextLogger 等）
    text event port logTextOut
    @ 指令分發器（指令分發器，Svc.CommandDispatcher / cmdDisp）
    command recv port cmdIn
    @ 指令分發器（指令分發器，Svc.CommandDispatcher / cmdDisp）— 註冊
    command reg port cmdRegOut
    @ 指令分發器（指令分發器，Svc.CommandDispatcher / cmdDisp）— 回應
    command resp port cmdResponseOut

    @ posValid = 3D fix 且 hdop <= 此門檻且 dataAge <= FIX_STALE_AGE_S（prmDb）
    param HDOP_MAX: F32 default 2.5
    @ SELFTEST 內需收到合法句子的 tick 數
    param SELFTEST_TICKS: U32 default 5
    @ READY 中連續無資料幾個 tick 發一次 NO_DATA 事件（不進 FAULT）
    param NODATA_WARN_TICKS: U32 default 2
    @ E4 完全逾時：READY 中靜默達此 tick 數發一次 NODATA_TIMEOUT（維持 READY；預設 10＝10 s@rg1Hz）
    param NODATA_TIMEOUT_TICKS: U32 default 10
    @ READY 中連續無資料幾個 tick 進 FAULT；0＝永不（C6 預設，飛行中失鎖／斷線不中止）
    param NODATA_FAULT_TICKS: U32 default 0
    @ 相同 GGA 連續重複幾個 tick 判定 STUCK（READY=降級 edge；SELFTEST=FAULT）
    param STUCK_TICKS: U32 default 3
    @ dataAge 超過此秒數強制 posValid=false（prmDb）
    param FIX_STALE_AGE_S: F32 default 1.5

    @ 握手狀態（只走遙測，不進 GnssFix）
    telemetry HsState: RocketTypes.DrvState
    telemetry FaultCount: U32
    telemetry BadChecksumCount: U32
    telemetry TruncatedCount: U32
    telemetry FixSeq: U32
    @ 最新輸出的 dataAge_s（秒）
    telemetry DataAge: F32
    @ 連續無 UART 資料的 tick 數
    telemetry NoDataTicks: U32
    @ 1＝目前處於失鎖 latch（已報 FIX_LOST、待 FIX_RECOVERED）
    telemetry FixLost: U8

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "GnssDriver {} -> {}"

    @ code：Gnss::GnssFault（NO_DATA/NODATA_TIMEOUT/BAD_CHECKSUM/TRUNCATED/STUCK/BUS_ERROR/SELFTEST_FAIL/FIX_LOST/FIX_RECOVERED）
    @ NO_DATA／NODATA_TIMEOUT／FIX_LOST／FIX_RECOVERED／STUCK(READY) 為降級 edge；SELFTEST 中 STUCK／BUS_ERROR／SELFTEST_FAIL 才進 FAULT
    event Fault(code: U8, count: U32) \
      severity warning high \
      format "GnssDriver fault code={} count={}"
  }

}
