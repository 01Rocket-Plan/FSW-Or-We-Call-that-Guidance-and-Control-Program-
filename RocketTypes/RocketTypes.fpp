# 01RocketPlan 共用型別（依介面合約 v0.1 與已核可勘誤）
# 慣例：世界系 ENU；機體系業界標準 X 指向鼻錐（滾轉）、Y 右（俯仰）、Z = X×Y（偏航）；四元數 Hamilton 純量在前 (w,x,y,z)；
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

  @ US MPU6050：感測器原生軸例外（只記錄）；accelSat/gyroSat 供紀錄組事後分析
  struct Mpu6050Sample {
    hdr: SampleHeader
    accel_sensor_mps2: [3] F32
    gyro_sensor_rads: [3] F32
    accelSat: bool
    gyroSat: bool
  }

  @ 氣壓樣本（只供速度／高度估計，不直接觸發轉移）
  struct BaroSample {
    hdr: SampleHeader
    press_Pa: F32
    temp_C: F32
    alt_m: F32
  }

  @ BME680 濕度：只送 Recorder（加熱器／氣體電阻已移除，見 PR #4）
  struct EnvSample {
    hdr: SampleHeader
    humidity_pct: F32
  }

  @ US DHT22 溫濕度：只送 Recorder（rg1Hz 每 2 tick 一筆；v2 定義，SI 單位）
  struct HumTempSample {
    hdr: SampleHeader
    temp_K: F32
    humidity_frac: F32
  }

  # =====================================================================
  # GNSS 組（梁承業）— GnssFix v0.2（首席工程師核定；已由 GNSS 組實作 GnssDriver／Sim）
  # 欄位：seq/t_s 在 SampleHeader；gpsTow_s、gpsWeek 僅記錄；velEnu U=NaN 若無垂直速度；
  # posValid = 3D 且 hdop<=prmDb；valid 僅 READY；fixOut 只接 tlmSend/sdLogger。
  # 型別欄位本段未改（與主幹一致）；僅加組別標註供審閱。
  # =====================================================================
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
  port Mpu6050Data(s: Mpu6050Sample)
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

  # =====================================================================
  # 提案 v0.3（MCU 組 EdwardWang+Claude，2026-11-10）：補 FlightMode 缺的介面
  # 對應 Components/MCU/FlightMode/README.md「缺的介面」FM-IN-2／3／5／7。
  # 只新增，不改既有型別；既有元件不受影響。
  # =====================================================================

  @ 哪一級
  enum Stage { DS = 0, US = 1 }

  @ 開傘指令（FM-IN-7）：flightMode → 開傘驅動。每級只開一次，fire-once 由驅動＋NvFlags 閂鎖。
  @ 機械計時器備援（逾時 +10 s）不經 MCU，不在這條路上。
  struct ChuteCmd {
    t_s: F64
    seq: U32
    stage: Stage
    @ true＝SEP_FAIL 時的下級側傘（SSS SIDE_CHUTE）；硬體同一顆就忽略
    side: bool
    valid: bool
  }

  @ 箭上離散訊號（FM-IN-2／3）：分離面微動開關、著地接觸。
  @ 「下令分離 ≠ 已經分開」：sepSwitchOpen 才是分開的證據（SSS §7）。
  struct DiscreteSample {
    hdr: SampleHeader
    @ 分離面開關已斷（兩級分開）
    sepSwitchOpen: bool
    @ 本級著地
    groundContact: bool
  }

  @ DS↔US 級間鏈路（FM-IN-5）：分離前 DS 告訴 US「已下令分離／禁上級傘」。
  @ 分離後斷線是預期行為；US 只信 alive=true 的資料（SSS _tick_us）。
  struct StageLinkMsg {
    t_s: F64
    seq: U32
    @ 對端還活著（逾時由接收端判斷）
    alive: bool
    @ DS 已下令分離
    sepCmd: bool
    @ DS 下令分離的時刻（<0＝沒有）；US 開傘計時的原點
    tSepCmd_s: F64
    @ 禁上級傘（假分／SEP_FAIL／ABORT）
    usInhibit: bool
  }

  port ChuteCmdPort(c: ChuteCmd)
  port DiscreteData(s: DiscreteSample)
  port StageLinkPort(m: StageLinkMsg)
}
