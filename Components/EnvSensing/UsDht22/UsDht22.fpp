module Components {

  @ DHT22 模擬器故障注入
  enum DhtSimFault { NONE, NO_RESPONSE, BAD_CHECKSUM, STUCK, OUT_OF_RANGE, SHORT_FRAME }

  @ US DHT22 溫濕度驅動（RMT 擷取）。接 rg1Hz：偶數 tick 觸發，下一個 tick 解碼，每 2 s 一筆，只送 Recorder。
  @ 平台 I/O 由拓樸呼叫 configure() 注入（ESP32 用 Dht22RmtEsp32）。
  passive component UsDht22 {

    @ rg1Hz
    sync input port schedIn: Svc.Sched

    @ 溫濕度樣本 → sdLogger
    output port humTempOut: RocketTypes.HumTempData

    time get port timeCaller
    telemetry port tlmOut
    event port logOut
    text event port logTextOut

    telemetry HsState: RocketTypes.DrvState
    telemetry ErrorCount: U32
    telemetry Seq: U32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "UsDht22 {} -> {}"

    @ code：1 NO_RESPONSE、2 CHECKSUM、3 OUT_OF_RANGE、4 STUCK、5 IO_INIT
    event Fault(code: U8, detail: U32) \
      severity warning high \
      format "UsDht22 fault code={} detail={}"
  }

  @ UsDht22 的 Sim 版（base id = UsDht22 + 0x80）
  passive component UsDht22Sim {

    sync input port schedIn: Svc.Sched
    output port humTempOut: RocketTypes.HumTempData

    time get port timeCaller
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 設定模擬溫濕度
    sync command SIM_SET(temp_C: F32, humidity_pct: F32)

    @ 注入故障
    sync command SIM_INJECT(fault: DhtSimFault)

    telemetry HsState: RocketTypes.DrvState
    telemetry ErrorCount: U32
    telemetry Seq: U32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "UsDht22Sim {} -> {}"

    event Fault(code: U8, detail: U32) \
      severity warning high \
      format "UsDht22Sim fault code={} detail={}"
  }

}
