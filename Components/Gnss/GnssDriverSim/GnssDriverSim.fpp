module Components {

  @ Sim 專用故障（對應 Gnss::GnssSimFault）
  enum GnssSimFault { NONE, NO_FIX, FIX_2D_ONLY, BAD_CHECKSUM, NO_DATA, STUCK, TRUNCATED }

  @ GnssDriver 的 Sim 版：同一個 GnssCore，接 NMEA 位元組流模擬器
  @ Base ID = GnssDriver(0x5000 per conventions；先前暫定 0x0A00) + 0x80 = 0x5080
  @ 埠與真驅動輸出側相同；schedIn 接 rg1Hz。安裝位置：DS／US 皆可。
  @ Gate C6：與真驅動相同——飛行中失鎖／斷線不中止，降級＋edge 事件＋可選 NODATA_FAULT_TICKS。
  passive component GnssDriverSim {

    @ 速率組 tick（rg1Hz／Svc.RateGroup）
    sync input port schedIn: Svc.Sched
    @ GnssFix v0.2 → tlmSend / sdLogger only
    output port fixOut: RocketTypes.GnssData

    @ F´ 時間服務
    time get port timeCaller
    @ 參數資料庫讀取（prmDb / Svc.PrmDb）
    param get port prmGetOut
    @ 參數資料庫寫入（prmDb / Svc.PrmDb）
    param set port prmSetOut
    @ 遙測輸出（tlmSend / Svc.TlmChan）
    telemetry port tlmOut
    @ 事件日誌（Svc.EventManager）
    event port logOut
    @ 文字事件
    text event port logTextOut
    @ 指令分發器（指令分發器，Svc.CommandDispatcher / cmdDisp）
    command recv port cmdIn
    @ 指令分發器（指令分發器，Svc.CommandDispatcher / cmdDisp）— 註冊
    command reg port cmdRegOut
    @ 指令分發器（指令分發器，Svc.CommandDispatcher / cmdDisp）— 回應
    command resp port cmdResponseOut

    @ posValid 的 HDOP 上限（與真驅動同名同預設）
    param HDOP_MAX: F32 default 2.5
    @ SELFTEST 內需收到合法句子的 tick 數
    param SELFTEST_TICKS: U32 default 5
    @ READY 中連續無資料幾個 tick 發一次 NO_DATA（不進 FAULT）
    param NODATA_WARN_TICKS: U32 default 2
    @ E4 完全逾時：靜默達此 tick 發一次 NODATA_TIMEOUT（維持 READY；預設 10）
    param NODATA_TIMEOUT_TICKS: U32 default 10
    @ READY 中連續無資料幾個 tick 進 FAULT；0＝永不（C6 預設）
    param NODATA_FAULT_TICKS: U32 default 0
    @ 相同 GGA 連續重複幾個 tick 判定 STUCK（READY=降級；SELFTEST=FAULT）
    param STUCK_TICKS: U32 default 3
    @ dataAge 超過此秒數強制 posValid=false
    param FIX_STALE_AGE_S: F32 default 1.5

    @ 設定軌跡：起點（度、公尺 MSL）與 ENU 速度（m/s）
    sync command SIM_SET_TRAJ(lat0_deg: F64, lon0_deg: F64, alt0_m: F32, velE_mps: F32, velN_mps: F32, velU_mps: F32)
    @ 雜訊標準差
    sync command SIM_SET_NOISE(posH_sigma_m: F32, posV_sigma_m: F32, vel_sigma_mps: F32)
    @ 輸出延遲與掉 fix 機率
    sync command SIM_SET_LATENCY(latency_s: F32, dropout_prob: F32)
    @ 注入故障（含 NO_FIX／NO_DATA 測 C6 失鎖／斷線）
    sync command SIM_INJECT(fault: GnssSimFault)
    @ HDOP 偏置（測試用：讓 hdop 可超過門檻）
    sync command SIM_SET_HDOP_BIAS(bias: F32)

    telemetry HsState: RocketTypes.DrvState
    telemetry FaultCount: U32
    telemetry FixSeq: U32
    telemetry DataAge: F32
    telemetry NoDataTicks: U32
    telemetry FixLost: U8

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "GnssDriverSim {} -> {}"

    @ code：同 GnssDriver（含 FIX_LOST／FIX_RECOVERED／NO_DATA／NODATA_TIMEOUT／READY-STUCK edge）
    event Fault(code: U8, count: U32) \
      severity warning high \
      format "GnssDriverSim fault code={} count={}"
  }
}
