@ MG996R interstage separation SEP_1–4 (DsMcu only). Identity via params model+role.
module Components {

  @ MG996R SEP_1–4 (DsMcu). Base ID 0x3100. Valid (model,role)=(MG996R,SEP) else FAULT.
  @ No cmd_timeout→neutral. Fire-once via Power.SepReleasedSet/Get. No sleep.
  passive component SepServoDriver {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    sync input port cmdIn: RocketTypes.SepCmdPort
    sync input port uvIn: RocketTypes.UvStatePort

    @ Power NvFlags named ports (module Power). We CALL these (output).
    output port sepReleasedGet: Power.SepReleasedGet
    output port sepReleasedSet: Power.SepReleasedSet

    @ Report channel state (Sim may connect)
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
    telemetry model: ServoModel
    telemetry role: ServoRole

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
    @ SELFTEST rejected (model,role); EnterFault(reason=3) also fired
    event ModelRoleMismatch(model: ServoModel, role: ServoRole) \
      severity warning high \
      format "SepServoDriver model/role mismatch model={} role={}"

    @ Hardware model — topology must set MG996R
    param model: ServoModel default ServoModel.MG996R
    @ Role — must be SEP
    param role: ServoRole default ServoRole.SEP
    param lock_us: U16 default MG996R_PULSE_MIN_US
    param fire_us: U16 default MG996R_PULSE_MAX_US
    param fire_hold_s: F32 default 0.5
    param pulse_min_us: U16 default MG996R_PULSE_MIN_US
    param pulse_max_us: U16 default MG996R_PULSE_MAX_US
    param selftest_deadline_s: F32 default 1.0
  }

}
