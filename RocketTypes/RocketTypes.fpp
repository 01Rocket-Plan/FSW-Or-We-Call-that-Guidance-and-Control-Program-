# 01RocketPlan 共用型別（依介面合約 v0.1 與已核可勘誤）
# 慣例：世界系 ENU；機體系 Z 沿長軸指向鼻錐、X／Y 橫向（右手系）；四元數 Hamilton 純量在前 (w,x,y,z)；
#       SI 單位，名稱帶後綴 _m _mps _mps2 _rad _rads _s _V _A。時間只取自 F´ 時間服務。
module RocketTypes {

  @ 共用樣本標頭：valid 只在驅動 READY 時可為 true
  struct SampleHeader {
    t_s: F64
    seq: U32
    valid: bool
  }

  @ 驅動握手狀態（只走遙測，不放進樣本）
  enum DrvState { OFF, INIT, SELFTEST, READY, FAULT }

  @ IMU 原始樣本（機體系）
  struct ImuSample {
    hdr: SampleHeader
    acc_mps2: [3] F32
    gyro_rads: [3] F32
    gyroSat: bool
  }

  @ US MPU6050：感測器原生軸例外（只記錄）
  struct Mpu6050Sample {
    hdr: SampleHeader
    accel_sensor_mps2: [3] F32
    gyro_sensor_rads: [3] F32
  }

  @ 氣壓樣本（只供速度／高度估計，不直接觸發轉移）
  struct BaroSample {
    hdr: SampleHeader
    press_Pa: F32
    temp_C: F32
    alt_m: F32
  }

  @ BME680 濕度／氣體：只送 Recorder
  struct EnvSample {
    hdr: SampleHeader
    humidity_pct: F32
    gas_ohm: F32
  }

  @ US DHT22 溫濕度：只送 Recorder（rg1Hz 每 2 tick 一筆；v2 定義，SI 單位）
  struct HumTempSample {
    hdr: SampleHeader
    temp_K: F32
    humidity_frac: F32
  }

  enum FixType { NO_FIX, FIX_2D, FIX_3D }

  @ GNSS 定位：只接 tlmSend、sdLogger，不進 estimator；缺垂直速度時 U 軸填 NaN
  struct GnssFix {
    hdr: SampleHeader
    gpsTow_s: F64
    gpsWeek: U16
    fixType: FixType
    numSats: U8
    hdop: F32
    lat_rad: F64
    lon_rad: F64
    altMsl_m: F32
    velEnu_mps: [3] F32
    posValid: bool
    velHValid: bool
    velUValid: bool
    dataAge_s: F32
  }

  @ 估態輸出 StateOut v1.0
  struct StateOut {
    seq: U32
    t_s: F64
    q_wxyz: [4] F32
    rate_body_rads: [3] F32
    tilt_rad: F32
    vel_enu_mps: [3] F32
    velValid: bool
    alt_m: F32
    baroAge_s: F32
    accAxial_mps2: F32
    imuValid: bool
    gyroSat: bool
  }

  @ 控制律輸出：四片尾翼機體系角度（rad）
  struct FinCmd {
    t_s: F64
    seq: U32
    fin_rad: [4] F32
    lockNeutral: bool
  }

  @ 舵機驅動指令（µs 只在 actuatorMgr 之後出現）
  struct ServoCmd {
    t_s: F64
    seq: U32
    pulse_us: [4] U16
    lockNeutral: bool
  }

  @ 級間分離指令
  struct SepCmd {
    t_s: F64
    seq: U32
    channelMask: U8
    valid: bool
  }

  @ 欠壓狀態（電源組只發事件，行為由 flightMode／actuatorMgr 決定）
  enum UvState { NOMINAL, UV_WARN, UV_SHED, UV_CRITICAL }

  @ 飛行模式列舉（僅介面；狀態機邏輯由 Edward 實作）
  enum FlightModeId {
    PAD_IDLE, ARMED, LIFTOFF, BOOST_PASSIVE, BOOST_ACTIVE, COAST, SEPARATED,
    LS_DESCENT, LS_CHUTE, LS_LEGS, LS_LANDED,
    US_COAST, US_CHUTE, US_LANDED, SEP_FAIL
  }

  port ImuData(s: ImuSample)
  port BaroData(s: BaroSample)
  port EnvData(s: EnvSample)
  port HumTempData(s: HumTempSample)
  port GnssData(fix: GnssFix)
  port StateData(s: StateOut)
  port FinCmdPort(c: FinCmd)
  port ServoCmdPort(c: ServoCmd)
  port SepCmdPort(c: SepCmd)
  port UvStatePort(st: UvState)
  port ModePort(mode: FlightModeId)
}
