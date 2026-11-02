module Power {

  @ Set-only OR of SEP_1..4 release bits (bit0..3); commit to NVS immediately.
  @ Callers (e.g. SepServoDriver) declare matching output ports of this type.
  port SepReleasedSet(mask: U8) -> Fw.Success

  @ Read RAM-cached SEP release mask (loaded from NVS at init; no flash I/O).
  port SepReleasedGet -> U8

  @ Set-only latch: 分離已下令 (separation commanded).
  port SepCommanded -> Fw.Success

  @ Set-only latch: DS 傘已下令 (DS parachute commanded).
  port DsChuteCommanded -> Fw.Success

  @ Read current phase value from RAM cache.
  port PhaseGet -> U32

  @ Set phase (dual-slot ping-pong + CRC). Formal param is phaseVal because
  @ "phase" is reserved in FPP.
  port PhaseSet(phaseVal: U32) -> Fw.Success

}
