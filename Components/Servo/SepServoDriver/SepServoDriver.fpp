# 元件指定：型號 MG996R 標準伺服 ×4，級間分離 SEP_1–4，僅 DsMcu。
# Base ID 0x3100，GPIO 32/33/13/4，LEDC 高速 ch4–7、timer0。
# lock_us 1000 / fire_us 2000；fire_hold_s 0.5 後保持 fire_us。
# 每通道一生只釋放一次（經 Power.SepReleasedSet/Get NvFlags）。
# 腳位為 MCU 組 v0.1 草案，待 Anatoly 確認，實際值以 Components/Servo/Pwm/ServoPwmPins.hpp 為準。
# 供電：供電依電力組定案（待確認）。
#
module Components {

  @ MG996R 級間分離 SEP_1–4（DsMcu）。Base ID 0x3100。
  @ cmd_timeout 回中位不適用；每通道一生只釋放一次（經 NvFlags）。不 sleep。
  passive component SepServoDriver {

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
