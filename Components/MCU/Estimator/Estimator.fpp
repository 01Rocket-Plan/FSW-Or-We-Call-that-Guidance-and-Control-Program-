module Mcu {

  @ DS 估態 W1：ImuData＋BaroData → StateData（StateOut v1.0）
  @ Base ID 0x1100（docs/conventions.md §5）。接 rg1kHz，排在 imuDriver 之後、controller 之前。
  @ 邏輯全在 EstimatorCore（純 C++，可主機測）；本元件只搬資料、取時間、發遙測。
  @ 輸入埠一律 guarded：氣壓（rg100Hz）與 1 kHz 鏈可能在不同核心。
  passive component Estimator {

    @ rg1kHz：每拍跑一次 EstimatorCore::step
    guarded input port schedIn: Svc.Sched

    @ imuDriver → 機體系比力／角速度
    guarded input port imuIn: RocketTypes.ImuData

    @ baroDriver → 氣壓高度
    guarded input port baroIn: RocketTypes.BaroData

    @ flightMode → 目前模式（只用來判斷「在不在墊上」）
    guarded input port modeIn: RocketTypes.ModePort

    @ → controller、flightMode、sdLogger、tlmSend（未接的不送）
    output port stateOut: [4] RocketTypes.StateData

    time get port timeCaller
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    # ------------------------------------------------------------ 遙測（10 Hz）
    telemetry aligned: bool
    telemetry onPad: bool
    telemetry tilt_rad: F32
    telemetry alt_m: F32
    telemetry velU_mps: F32
    telemetry accAxial_mps2: F32
    telemetry baroAge_s: F32
    telemetry gyroBiasX_rads: F32
    telemetry gyroBiasY_rads: F32
    telemetry gyroBiasZ_rads: F32
    telemetry baroRejectCount: U32
    telemetry imuStaleCount: U32

    # ------------------------------------------------------------ 事件
    event Aligned(tilt_rad: F32) severity activity high format "Estimator 墊上對準完成 tilt_rad={}"
    event ImuStale() severity warning high format "Estimator IMU 不新鮮（> imuStale_s）"
    event ImuRecovered() severity activity high format "Estimator IMU 恢復"
    event LeftPad() severity activity high format "Estimator 離開墊上模式：停止水平修正、凍結零偏與氣壓基準"

    # ------------------------------------------------------------ 參數（預設＝README 表）
    param g_mps2: F32 default 9.80665
    param padAlign_s: F32 default 0.5
    param accGate_mps2: F32 default 0.5
    param rateGate_rads: F32 default 0.05
    param kLevel_rads: F32 default 0.5
    param gyroBiasTau_s: F32 default 2.0
    param qAcc_m2ps4: F32 default 0.25
    param rBaro_m2: F32 default 0.25
    param baroGate_m: F32 default 10.0
    param rZupt_m2ps2: F32 default 0.0001
    param altRefTau_s: F32 default 5.0
    param imuStale_s: F32 default 0.020
    param baroMaxAge_s: F32 default 1.0
    param dtMax_s: F32 default 0.005
  }

}
