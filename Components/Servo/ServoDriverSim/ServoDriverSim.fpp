# 元件指定（Sim）：模擬 SG90 伺服 ServoDriver（DsMcu 尾翼 TAIL_1–4／UsMcu 雲台 PITCH/ROLL）；Base ID servoDriverSim 0x3080、gimbalServoDriverSim 0x3280（channel_count=2）；無硬體 I/O。
module Components {

  @ ServoDriver 的 Sim 版（埠集合與真驅動相同）。Base ID：servoDriverSim 0x3080、gimbalServoDriverSim 0x3280（channel_count=2）。
  @ 核心邏輯在 Components/Servo/SimCore（host 可測）。pwmOut 送出實際採用的 µs（夾限／回中立後）。
  @ 接 rg1kHz；內部依 pwm_hz（預設 50）零階保持。µs 硬限 pulse_min/max。不 sleep、時間只取自 F´ 時間服務。
  passive component ServoDriverSim {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    @ actuatorMgr → 本驅動（合約 cmdIn）
    sync input port cmdIn: RocketTypes.ServoCmdPort
    @ 欠壓狀態：只記事件，不動舵（E6）
    sync input port uvIn: RocketTypes.UvStatePort

    @ 實際輸出 µs（Sim 拓樸可接；真機可懸空）
    output port pwmOut: RocketTypes.ServoCmdPort

    command recv port cmdDispIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry drvState: RocketTypes.DrvState
    telemetry lastSeq: U32
    telemetry lastPulse0_us: U16
    telemetry lastPulse1_us: U16
    telemetry lastPulse2_us: U16
    telemetry lastPulse3_us: U16
    telemetry cmdAge_s: F32
    telemetry uvState: RocketTypes.UvState
    telemetry clampCount: U32
    telemetry dropCount: U32

    event EnterReady() severity activity high format "ServoDriver READY"
    event EnterFault(reason: U8) severity warning high format "ServoDriver FAULT reason={}"
    event CmdDropped(seq: U32, reason: U8) severity warning low format "ServoCmd drop seq={} reason={}"
    event PulseClamped(ch: U8, raw: U16, used: U16) severity warning low format "ch{} clamp {}→{}"
    event CmdTimeout(age_s: F32) severity warning high format "ServoCmd timeout age_s={}"
    event LockNeutral(source: U8) severity activity high format "lockNeutral source={}"
    event UvSeen(st: RocketTypes.UvState) severity activity high format "UvState={}"

    param neutral_us: U16 default 1500
    param pulse_min_us: U16 default 1000
    param pulse_max_us: U16 default 2000
    param cmd_timeout_s: F32 default 0.05
    param pwm_hz: U16 default 50
    param channel_count: U8 default 4
    param selftest_deadline_s: F32 default 1.0
  }

}
