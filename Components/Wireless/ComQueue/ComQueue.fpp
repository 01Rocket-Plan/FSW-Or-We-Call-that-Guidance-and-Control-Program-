module Components {

  @ 下行優先層（P0 最高；分層表見 README，Edward 2026-11-08 定案）
  enum ComPriority : U8 {
    @ 關鍵事件＋GNSS fix：模式轉移、指令被拒、ABORT 回覆、SEP 分離、開傘、IMU 失效、低壓卸載；
    @ GNSS fix 只留最新一筆（新 fix 覆蓋未送出的舊 fix）
    P0 = 0
    @ 導航：Estimator 姿態／傾角／速度、IMU
    P1 = 1
    @ 健康：flightMode 週期、電源三軌、舵面指令角
    P2 = 2
    @ 診斷：handshake、佇列 HWM／丟包數、GNSS data age
    P3 = 3
    @ 可犧牲：相機縮圖／分塊、除錯文字事件
    P4 = 4
  }

  @ comQueue：P0..P4 嚴格優先 FIFO；Base ID 0x8200
  @ passive＋guarded comIn（多 rate group 入列，元件 mutex）；run＝sync，僅掛 Core 0 rg100Hz
  @ 每 run 最多吐 maxPktsPerRun 幀；全層滿丟最新（登記為只留最新的 apid 除外）；comStatusIn 忙碌不推進
  @ Viktor：若拓樸量測顯示 rg 預算爆掉，再改 active
  passive component ComQueue {

    @ 各優先層入列（多元件／不同 rate group → guarded 互斥）
    guarded input port comIn: [5] Svc.ComDataWithContext

    @ Core 0 rg100Hz；每拍最多 maxPktsPerRun 包
    sync input port run: Svc.Sched

    @ 下一幀 → framer.comIn
    output port buffOut: Svc.ComDataWithContext

    @ rfDriver 忙碌／就緒
    @ F´ v4：通訊狀態埠是 Fw.SuccessCondition（舊 stub 的 Svc.ComStatus 在真 F´ 不存在）
    sync input port comStatusIn: Fw.SuccessCondition

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
