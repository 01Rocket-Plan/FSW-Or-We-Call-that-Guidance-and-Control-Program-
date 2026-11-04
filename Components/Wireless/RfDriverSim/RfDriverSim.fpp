module Components {

  @ RF 數傳 Sim；Base ID 0x8480（＝0x8400+0x80；首席已核准）
  @ 埠集合與 RfDriver 相同，可替換拓樸實例；不宣告硬體型號／角色。
  @ 不得讀取任何 truth／飛行狀態；掉包率／延遲／BER 由 prm 注入。
  passive component RfDriverSim {

    sync input port dataIn: Svc.ComDataWithContext
    output port dataOut: Svc.ComDataWithContext
    output port comStatusOut: Svc.ComStatus
    output port healthOut: RfHealth

    sync input port run: Svc.Sched

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    param channel: U32 default 0
    param power_dBm: I32 default 0
    param txTimeout_s: F64 default 1.0
    param initDeadline_s: F64 default 2.0
    param selftestDeadline_s: F64 default 2.0
    param readyDeadline_s: F64 default 5.0
    param txTimeoutLimit: U32 default 3

    param dropRate: F64 default 0.0
    param latencyTicks: U32 default 0
    param ber: F64 default 0.0

    telemetry State: RocketTypes.DrvState
    telemetry TxTimeoutCount: U32
    telemetry TxDroppedCount: U32
    telemetry SentFrames: U32
    telemetry SimDropCount: U32

    event RfState(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high format "RfDriverSim {} -> {}"
    event RfFault(code: U32) severity warning high format "RfDriverSim FAULT code={}"
    event RfTxTimeout(len: U32) severity warning low format "RfDriverSim TX timeout len={}"
  }

}
