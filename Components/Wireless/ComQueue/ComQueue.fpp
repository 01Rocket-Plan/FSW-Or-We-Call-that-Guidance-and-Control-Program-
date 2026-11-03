module Components {

  @ 下行優先層（P0 最高）
  enum ComPriority : U8 { P0 = 0, P1 = 1, P2 = 2, P3 = 3, P4 = 4 }

  @ comQueue：P0..P4 嚴格優先 FIFO；Base ID 0x8200；rg100Hz／Core 0（合約稱主動；本殼以 sync run 掛率組，待拓樸可改 async）
  @ 永不阻塞；P0 滿丟最舊、其餘丟最新；comStatusIn 忙碌時不推進。
  passive component ComQueue {

    @ 各優先層入列（拓樸可連多個來源到不同 index）
    sync input port comIn: [5] Svc.ComDataWithContext

    @ rg100Hz
    sync input port run: Svc.Sched

    @ 下一幀 → framer.comIn
    output port buffOut: Svc.ComDataWithContext

    @ rfDriver 忙碌／就緒
    sync input port comStatusIn: Svc.ComStatus

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 每層佇列深度（prmDb）
    param depthP0: U8 default 8
    param depthP1: U8 default 8
    param depthP2: U8 default 8
    param depthP3: U8 default 4
    param depthP4: U8 default 4

    telemetry QueueHwmP0: U8
    telemetry QueueHwmP1: U8
    telemetry QueueHwmP2: U8
    telemetry QueueHwmP3: U8
    telemetry QueueHwmP4: U8
    telemetry DropCountTotal: U32
    telemetry HighWater: U8

    @ EVT_QUEUE_FULL_DROP（≤1/s，附累計數）
    event QueueFullDrop(pri: ComPriority, dropCount: U32) \
      severity warning low \
      format "comQueue full drop pri={} count={}"

    event HighWaterMark() \
      severity activity low \
      format "comQueue high water"
  }

}
