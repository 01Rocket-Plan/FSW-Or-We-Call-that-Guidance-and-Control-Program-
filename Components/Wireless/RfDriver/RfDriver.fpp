module Components {

  @ 射頻硬體型號 — 主飛行鏈路：LoRa Ra-02（433 MHz），晶片 Semtech SX1278
  @ 備註（非本 enum 取值／本 PR 不實作）：HC-12（SI4438 433 MHz）＝備援鏈；
  @ NRF24L01（2.4 GHz）＝地面站／發射台用，不上箭（Edward 設計文件）
  enum RfHwModel : U8 { LORA_RA_02 = 0 }

  @ 射頻角色：AIR_GROUND_LINK＝箭地無線鏈路（主飛行鏈）
  enum RfHwRole : U8 { AIR_GROUND_LINK = 0 }

  @ RF 數傳真驅動；Base ID 0x8400（首席已核准）
  @ 硬體型號 LORA_RA_02（LoRa Ra-02，433 MHz，Semtech SX1278，主飛行鏈）；角色＝AIR_GROUND_LINK
  @ 握手 OFF→INIT→SELFTEST→READY／FAULT；連續 txTimeout → FAULT
  passive component RfDriver {

    sync input port dataIn: Svc.ComDataWithContext
    output port dataOut: Svc.ComDataWithContext
    @ F´ v4：通訊狀態埠是 Fw.SuccessCondition（舊 stub 的 Svc.ComStatus 在真 F´ 不存在）
    output port comStatusOut: Fw.SuccessCondition
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

    @ 硬體型號 — LoRa Ra-02（433 MHz），晶片 Semtech SX1278，主飛行鏈
    param model: RfHwModel default RfHwModel.LORA_RA_02
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
