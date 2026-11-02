module Components {

  @ Sim 專用故障（對應 Gnss::GnssSimFault）
  enum GnssSimFault { NONE, NO_FIX, FIX_2D_ONLY, BAD_CHECKSUM, NO_DATA, STUCK, TRUNCATED }

  @ GnssDriver 的 Sim 版：同一個 GnssCore，接 NMEA 位元組流模擬器
  @ Base ID = GnssDriver(0x5000 per conventions；先前暫定 0x0A00) + 0x80 = 0x5080
  @ 埠與真驅動輸出側相同；schedIn 接 rg1Hz。安裝位置：DS／US 皆可。
  passive component GnssDriverSim {

    sync input port schedIn: Svc.Sched
    output port fixOut: RocketTypes.GnssData

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ posValid 的 HDOP 上限（與真驅動同名同預設）
    param HDOP_MAX: F32 default 2.5
    @ SELFTEST 內需收到合法句子的 tick 數
    param SELFTEST_TICKS: U32 default 5
    @ READY 中連續無資料幾個 tick 進 FAULT
    param NODATA_FAULT_TICKS: U32 default 5
    @ 相同 GGA 連續重複幾個 tick 判定 STUCK
    param STUCK_TICKS: U32 default 3

    @ 設定軌跡：起點（度、公尺 MSL）與 ENU 速度（m/s）
    sync command SIM_SET_TRAJ(lat0_deg: F64, lon0_deg: F64, alt0_m: F32, velE_mps: F32, velN_mps: F32, velU_mps: F32)
    @ 雜訊標準差
    sync command SIM_SET_NOISE(posH_sigma_m: F32, posV_sigma_m: F32, vel_sigma_mps: F32)
    @ 輸出延遲與掉 fix 機率
    sync command SIM_SET_LATENCY(latency_s: F32, dropout_prob: F32)
    @ 注入故障
    sync command SIM_INJECT(fault: GnssSimFault)
    @ HDOP 偏置（測試用：讓 hdop 可超過門檻）
    sync command SIM_SET_HDOP_BIAS(bias: F32)

    telemetry HsState: RocketTypes.DrvState
    telemetry FaultCount: U32
    telemetry FixSeq: U32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "GnssDriverSim {} -> {}"

    event Fault(code: U8, count: U32) \
      severity warning high \
      format "GnssDriverSim fault code={} count={}"
  }
}
