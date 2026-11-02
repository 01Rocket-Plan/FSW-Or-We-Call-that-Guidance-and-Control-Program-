module Components {

  @ Sim 專用故障（對應 Gnss::GnssSimFault；與驅動 GnssFault 分開）
  @ 用意：在位元組層注入，驗證 Core 失鎖／校驗／截斷路徑，而非直接改 Core 狀態。
  enum GnssSimFault { NONE, NO_FIX, FIX_2D_ONLY, BAD_CHECKSUM, NO_DATA, STUCK, TRUNCATED }

  @ GnssDriver Sim：同一 GnssCore + NMEA 位元組流。Base ID 0x5080。
  @ schedIn＝rg100Hz；失鎖表／整數分頻／emit／seq／C6 與真驅動相同（Edward 2026-11-02）。
  @ 為何共用 Core：Sim 與真機行為一致，host T1–T12 結論可外推到飛行軟體。
  passive component GnssDriverSim {

    @ 速率組 tick（rg100Hz）；內部分頻＋先 advance(dt) 再生產 NMEA 再 tick Core
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

    @ 以下 prm 語意／單位與 GnssDriver 相同（見 GnssDriver.fpp 註解）
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

    @ 設定模擬軌跡起點（度／m）與固定 ENU 速度（m/s）
    sync command SIM_SET_TRAJ(lat0_deg: F64, lon0_deg: F64, alt0_m: F32, velE_mps: F32, velN_mps: F32, velU_mps: F32)
    @ 量測雜訊 σ：水平／垂直位置（m）、水平速度（m/s）
    sync command SIM_SET_NOISE(posH_sigma_m: F32, posV_sigma_m: F32, vel_sigma_mps: F32)
    @ 輸出延遲（s）與每 epoch 掉 fix 機率（0~1）
    sync command SIM_SET_LATENCY(latency_s: F32, dropout_prob: F32)
    @ 注入位元組層故障（見 GnssSimFault）
    sync command SIM_INJECT(fault: GnssSimFault)
    @ HDOP 偏置（測超門檻）；最終仍夾在合理範圍
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
