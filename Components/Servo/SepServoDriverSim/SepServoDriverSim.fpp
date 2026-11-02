@ Sim shell for SepServoDriver (same ports/params/events). Core: Components/Servo/SimCore.
module Components {

  @ SepServoDriver Sim. Base ID 0x3180. Valid (model,role)=(MG996R,SEP). No hardware I/O.
  passive component SepServoDriverSim {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    sync input port cmdIn: RocketTypes.SepCmdPort
    sync input port uvIn: RocketTypes.UvStatePort

    output port sepReleasedGet: Power.SepReleasedGet
    output port sepReleasedSet: Power.SepReleasedSet

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
    event SepFaultHold(mask: U8) severity warning high format "SEP fault hold mask={}"
    event SepNvWriteFail(ch: U8) severity warning high format "SEP_{} nv write FAIL (held lock)"
    event UvSeen(st: RocketTypes.UvState) severity activity high format "UvState={}"
    event ModelRoleMismatch(model: ServoModel, role: ServoRole) \
      severity warning high \
      format "SepServoDriverSim model/role mismatch model={} role={}"

    param model: ServoModel default ServoModel.MG996R
    param role: ServoRole default ServoRole.SEP
    param lock_us: U16 default MG996R_PULSE_MIN_US
    param fire_us: U16 default MG996R_PULSE_MAX_US
    param fire_hold_s: F32 default 0.5
    param pulse_min_us: U16 default MG996R_PULSE_MIN_US
    param pulse_max_us: U16 default MG996R_PULSE_MAX_US
    param selftest_deadline_s: F32 default 1.0
  }

}
