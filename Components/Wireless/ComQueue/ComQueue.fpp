module Components {

  @ 下行優先層（P0 最高）
  enum ComPriority : U8 { P0 = 0, P1 = 1, P2 = 2, P3 = 3, P4 = 4 }

  @ comQueue：P0..P4 嚴格優先 FIFO；Base ID 0x8200
  @ passive＋guarded comIn（多 rate group 入列，元件 mutex）；run＝sync，僅掛 Core 0 rg100Hz
  @ 每 run 最多吐 maxPktsPerRun 幀；P0 滿丟最舊、其餘丟最新；comStatusIn 忙碌不推進
  @ Viktor：若拓樸量測顯示 rg 預算爆掉，再改 active
  passive component ComQueue {

    @ 各優先層入列（多元件／不同 rate group → guarded 互斥）
    guarded input port comIn: [5] Svc.ComDataWithContext

    @ Core 0 rg100Hz；每拍最多 maxPktsPerRun 包
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

    param depthP0: U8 default 8
    param depthP1: U8 default 8
    param depthP2: U8 default 8
    param depthP3: U8 default 4
    param depthP4: U8 default 4

    @ 單次 run 最多送出的封包數（≥1）
    param maxPktsPerRun: U8 default 4

    telemetry QueueHwmP0: U8
    telemetry QueueHwmP1: U8
    telemetry QueueHwmP2: U8
    telemetry QueueHwmP3: U8
    telemetry QueueHwmP4: U8
    telemetry DropCountTotal: U32
    telemetry HighWater: U8

    event QueueFullDrop(pri: ComPriority, dropCount: U32) \
      severity warning low \
      format "comQueue full drop pri={} count={}"

    event HighWaterMark() \
      severity activity low \
      format "comQueue high water"
  }

}
