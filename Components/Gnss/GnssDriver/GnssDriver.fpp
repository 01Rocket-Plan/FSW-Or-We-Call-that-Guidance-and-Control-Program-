module Components {

  @ TEL0157 GNSS 真驅動（UART NMEA：GGA/RMC/VTG/GSA）
  @ 安裝位置：DS／US 皆可（由拓撲決定）；fixOut 只接 tlmSend 與 sdLogger，不進 estimator。
  @ Base ID：conventions.md 為 0x5000；先前暫定 0x0A00，拓撲請以 conventions 為準。Sim = base+0x80。
  @ schedIn 接 rg1Hz；不 sleep、不讀系統時間；每 tick 最多一次 UART 讀取。
  passive component GnssDriver {

    sync input port schedIn: Svc.Sched

    @ GnssFix v0.2 → tlmSend / sdLogger only
    output port fixOut: RocketTypes.GnssData

    @ UART 位元組推入（ByteStream 驅動 → 本元件 ring buffer）；核心經 IGnssUart 每 tick 讀一次
    sync input port uartRecv: Drv.ByteStreamData
    @ 歸還 uartRecv 緩衝區所有權
    output port uartRecvReturn: Fw.BufferSend

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ posValid = 3D fix 且 hdop <= 此門檻（prmDb）
    param HDOP_MAX: F32 default 2.5
    @ SELFTEST 內需收到合法句子的 tick 數
    param SELFTEST_TICKS: U32 default 5
    @ READY 中連續無資料幾個 tick 進 FAULT
    param NODATA_FAULT_TICKS: U32 default 5
    @ 相同 GGA 連續重複幾個 tick 判定 STUCK
    param STUCK_TICKS: U32 default 3

    @ 握手狀態（只走遙測，不進 GnssFix）
    telemetry HsState: RocketTypes.DrvState
    telemetry FaultCount: U32
    telemetry BadChecksumCount: U32
    telemetry TruncatedCount: U32
    telemetry FixSeq: U32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "GnssDriver {} -> {}"

    @ code：見 Gnss::GnssFault（NO_DATA/BAD_CHECKSUM/TRUNCATED/STUCK/BUS_ERROR/SELFTEST_FAIL）
    event Fault(code: U8, count: U32) \
      severity warning high \
      format "GnssDriver fault code={} count={}"
  }

}
