module Components {

  @ Sim 專用故障（對應 Gnss::GnssSimFault）
  enum GnssSimFault { NONE, NO_FIX, FIX_2D_ONLY, BAD_CHECKSUM, NO_DATA, STUCK, TRUNCATED }

  @ GnssDriver Sim：同一 GnssCore + NMEA 位元組流。Base ID 0x5080。
  @ schedIn＝rg100Hz；失鎖表與真驅動相同（Edward 2026-11-02）。
  passive component GnssDriverSim {

    @ 速率組 tick（rg100Hz）
    sync input port schedIn: Svc.Sched
    @ GnssFix v0.2 → tlmSend / sdLogger only
    output port fixOut: RocketTypes.GnssData

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    @ 指令分發器（Svc.CommandDispatcher / cmdDisp）
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    param HDOP_MAX: F32 default 2.5
    param POLL_TRACKING_MS: U32 default 100
    param POLL_REACQ_FAST_MS: U32 default 100
    param POLL_REACQ_MED_MS: U32 default 1000
    param POLL_REACQ_SLOW_MS: U32 default 2000
    param REACQ_FAST_S: F32 default 15.0
    param REACQ_MED_S: F32 default 10.0
    param REACQ_SLOW_S: F32 default 30.0
    param INIT_ACQ_TIMEOUT_S: F32 default 300.0
    param SELFTEST_TIMEOUT_S: F32 default 5.0
    param LOSS_NO_GGA_S: F32 default 2.0
    param STUCK_POLLS: U32 default 3

    sync command SIM_SET_TRAJ(lat0_deg: F64, lon0_deg: F64, alt0_m: F32, velE_mps: F32, velN_mps: F32, velU_mps: F32)
    sync command SIM_SET_NOISE(posH_sigma_m: F32, posV_sigma_m: F32, vel_sigma_mps: F32)
    sync command SIM_SET_LATENCY(latency_s: F32, dropout_prob: F32)
    sync command SIM_INJECT(fault: GnssSimFault)
    sync command SIM_SET_HDOP_BIAS(bias: F32)

    telemetry HsState: RocketTypes.DrvState
    telemetry LockStage: GnssLockStage
    telemetry FaultCount: U32
    telemetry FixSeq: U32
    telemetry DataAge: F32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "GnssDriverSim {} -> {}"

    event LockStageChanged(fromStage: GnssLockStage, toStage: GnssLockStage) \
      severity activity high \
      format "GnssDriverSim lock {} -> {}"

    event FixFailedOpenSky \
      severity warning high \
      format "請檢察環境開放條件以利天線運作"

    event Fault(code: U8, count: U32) \
      severity warning high \
      format "GnssDriverSim fault code={} count={}"
  }
}
