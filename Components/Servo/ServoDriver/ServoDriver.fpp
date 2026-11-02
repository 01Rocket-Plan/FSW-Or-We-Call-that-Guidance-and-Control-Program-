@ SG90 PWM driver (TAIL on DsMcu / GIMBAL instance on UsMcu). Identity via params model+role.
module Components {

  @ SG90 tail (DS) / gimbal (US instance) PWM driver. Base ID 0x3000; gimbal instance 0x3200.
  @ Connect to rg1kHz; internal ZOH at pwm_hz (default SG90_PWM_HZ). Clamp to pulse_min/max.
  @ No sleep; time only from F´ time service. Valid (model,role,channel_count):
  @   (SG90, TAIL, 4) or (SG90, GIMBAL, 2) — else SELFTEST → FAULT + ModelRoleMismatch.
  passive component ServoDriver {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    @ actuatorMgr → this driver
    sync input port cmdIn: RocketTypes.ServoCmdPort
    @ Undervoltage: event only, no motion change
    sync input port uvIn: RocketTypes.UvStatePort

    @ Actual output µs (Sim topology may connect; flight may leave open)
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
    @ Formal identity (mirrors params; written every tlm cycle and on READY)
    telemetry model: ServoModel
    telemetry role: ServoRole

    event EnterReady() severity activity high format "ServoDriver READY"
    event EnterFault(reason: U8) severity warning high format "ServoDriver FAULT reason={}"
    event CmdDropped(seq: U32, reason: U8) severity warning low format "ServoCmd drop seq={} reason={}"
    event PulseClamped(ch: U8, raw: U16, used: U16) severity warning low format "ch{} clamp {}→{}"
    event CmdTimeout(age_s: F32) severity warning high format "ServoCmd timeout age_s={}"
    event LockNeutral(source: U8) severity activity high format "lockNeutral source={}"
    event UvSeen(st: RocketTypes.UvState) severity activity high format "UvState={}"
    @ SELFTEST rejected (model,role,channel_count); EnterFault(reason=3) also fired
    event ModelRoleMismatch(model: ServoModel, role: ServoRole, channel_count: U8) \
      severity warning high \
      format "ServoDriver model/role mismatch model={} role={} ch={}"

    @ Hardware model — topology must set SG90
    param model: ServoModel default ServoModel.SG90
    @ Role — TAIL (DS, 4 ch) or GIMBAL (US, 2 ch)
    param role: ServoRole default ServoRole.TAIL
    param neutral_us: U16 default SG90_NEUTRAL_US
    param pulse_min_us: U16 default SG90_PULSE_MIN_US
    param pulse_max_us: U16 default SG90_PULSE_MAX_US
    param cmd_timeout_s: F32 default 0.05
    param pwm_hz: U16 default SG90_PWM_HZ
    param channel_count: U8 default 4
    param selftest_deadline_s: F32 default 1.0
  }

}
