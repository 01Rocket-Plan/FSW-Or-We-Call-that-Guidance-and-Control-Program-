module Components {

  @ 藍牙備用傳輸（BLE）Sim；Base ID 0x8880（＝0x8800+0x80）
  @ 埠集合與 BleDriver／RfDriver 相同，可替換拓樸實例；不宣告硬體型號／角色。
  @ 不得讀取任何 truth／飛行狀態；連線／斷線／MTU／HAL 緩衝由 prm 注入（BleHalSim）。
  passive component BleDriverSim {

    sync input port dataIn: Svc.ComDataWithContext
    output port dataOut: Svc.ComDataWithContext
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

    param txTimeout_s: F64 default 1.0
    param initDeadline_s: F64 default 2.0
    param selftestDeadline_s: F64 default 2.0
    param readyDeadline_s: F64 default 5.0
    param txTimeoutLimit: U32 default 3

    @ 模擬：協商後 ATT_MTU（23..247）
    param simAttMtu: U16 default 247
    @ 模擬：HAL／堆疊每拍最多收幾個 notification（滿了 notify 失敗、下一拍續送）
    param simTxSlotsPerTick: U32 default 8
    @ 模擬：連上後每 N 拍自動斷一次（0＝不斷）
    param simLinkDropEveryTicks: U32 default 0
    @ 模擬：斷線後 N 拍自動重連（0＝不自動重連）
    param simReconnectAfterTicks: U32 default 0

    telemetry State: RocketTypes.DrvState
    telemetry TxDroppedCount: U32
    telemetry SentFrames: U32
    telemetry LinkUp: bool
    telemetry LinkDownCount: U32

    event RfState(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high format "BleDriverSim {} -> {}"
    event RfFault(code: U32) severity warning high format "BleDriverSim FAULT code={}"
  }

}
