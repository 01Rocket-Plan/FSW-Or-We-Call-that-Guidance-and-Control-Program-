# 元件指定（Sim）：模擬 MG996R 級間分離伺服 SepServoDriver（DsMcu SEP_1–4）；Base ID 0x3180；無硬體 I/O。
module Components {

  @ SepServoDriver 的 Sim 版（埠集合與真驅動相同）。Base ID 0x3180。核心在 Components/Servo/SimCore。
  @ cmd_timeout 回中位不適用；每通道一生只釋放一次（經 NvFlags）。不 sleep。
  passive component SepServoDriverSim {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    sync input port cmdIn: RocketTypes.SepCmdPort
    sync input port uvIn: RocketTypes.UvStatePort

    @ Power NvFlags named ports (module Power). We CALL these (output).
    output port sepReleasedGet: Power.SepReleasedGet
    output port sepReleasedSet: Power.SepReleasedSet

    @ 回報實際通道狀態（Sim 可接）
    output port pwmOut: RocketTypes.SepCmdPort

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
    telemetry firedMask: U8
    telemetry lockedMask: U8
    telemetry uvState: RocketTypes.UvState

    event EnterReady() severity activity high format "SepServoDriver READY"
    event EnterFault(reason: U8) severity warning high format "SepServo FAULT reason={}"
    event CmdDropped(seq: U32, reason: U8) severity warning low format "SepCmd drop seq={} reason={}"
    event SepFire(ch: U8) severity activity high format "SEP_{} fire"
    event SepAlreadyFired(ch: U8) severity warning low format "SEP_{} already fired (nv)"
    @ Only when driver enters FAULT — hold unreleased channels at lock_us
    event SepFaultHold(mask: U8) severity warning high format "SEP fault hold mask={}"
    @ NVS write failed for this channel bit; do NOT FAULT; keep lock_us; later cmd may retry
    event SepNvWriteFail(ch: U8) severity warning high format "SEP_{} nv write FAIL (held lock)"
    event UvSeen(st: RocketTypes.UvState) severity activity high format "UvState={}"

    param lock_us: U16 default 1000
    param fire_us: U16 default 2000
    param fire_hold_s: F32 default 0.5
    param pulse_min_us: U16 default 1000
    param pulse_max_us: U16 default 2000
    param selftest_deadline_s: F32 default 1.0
  }

}
