module Components {

  @ 射頻硬體型號（尚未定案 → UNASSIGNED；待 Edward 定）
  enum RfHwModel : U8 { UNASSIGNED = 0 }

  @ 射頻角色：AIR_GROUND_LINK＝箭地無線鏈路
  enum RfHwRole : U8 { AIR_GROUND_LINK = 0 }

  @ RF 數傳真驅動；Base ID 0x8400（首席已核准）
  @ 硬體型號 UNASSIGNED（TBD／待 Edward 定）；角色＝箭地無線鏈路（AIR_GROUND_LINK）
  @ 握手 OFF→INIT→SELFTEST→READY／FAULT；連續 txTimeout → FAULT
  passive component RfDriver {

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

    @ 硬體型號 — UNASSIGNED（射頻型號 TBD，待 Edward 定）
    param model: RfHwModel default RfHwModel.UNASSIGNED
    @ 角色 — 箭地無線鏈路
    param role: RfHwRole default RfHwRole.AIR_GROUND_LINK

    param channel: U32 default 0
    param power_dBm: I32 default 0
    param txTimeout_s: F64 default 1.0
    param initDeadline_s: F64 default 2.0
    param selftestDeadline_s: F64 default 2.0
    param readyDeadline_s: F64 default 5.0
    param txTimeoutLimit: U32 default 3

    telemetry State: RocketTypes.DrvState
    telemetry TxTimeoutCount: U32
    telemetry TxDroppedCount: U32
    telemetry SentFrames: U32
    telemetry Model: RfHwModel
    telemetry Role: RfHwRole

    event RfState(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high format "RfDriver {} -> {}"
    event RfFault(code: U32) severity warning high format "RfDriver FAULT code={}"
    event RfTxTimeout(len: U32) severity warning low format "RfDriver TX timeout len={}"
    event InitOk() severity activity low format "RfDriver INIT OK"
    event SelftestOk() severity activity low format "RfDriver SELFTEST OK"
    event SelftestFail(tries: U32) severity warning high format "RfDriver SELFTEST fail tries={}"
  }

}
