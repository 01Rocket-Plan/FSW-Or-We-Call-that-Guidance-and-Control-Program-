module Mcu {

  @ Controller 為何沒閉環（遙測）；數值對齊 ControllerCore.hpp HoldReason
  enum CtlHoldReason {
    NONE = 0
    NO_MODE = 1
    MODE_INACTIVE = 2
    NO_STATE = 3
    STATE_STALE = 4
    IMU_INVALID = 5
    DEMO_FIXED = 6
  }

  @ DS 姿控 W3：StateData＋ModePort → FinCmdPort
  @ Base ID 0x1200（docs/conventions.md §5）。接 rg1kHz，排在 estimator 之後、actuatorMgr 之前。
  @ 只在 BOOST_ACTIVE／COAST 且姿態可信、資料新鮮時閉環；其餘四片 0＋lockNeutral（SSS _authority）。
  @ 邏輯全在 ControllerCore（純 C++）。
  passive component Controller {

    @ rg1kHz
    guarded input port schedIn: Svc.Sched

    @ estimator → 姿態／角速度
    guarded input port stateIn: RocketTypes.StateData

    @ flightMode → 目前模式
    guarded input port modeIn: RocketTypes.ModePort

    @ → actuatorMgr
    output port finCmdOut: RocketTypes.FinCmdPort

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
    telemetry closedLoop: bool
    telemetry holdReason: CtlHoldReason
    telemetry errX_rad: F32
    telemetry errY_rad: F32
    telemetry fin1_rad: F32
    telemetry fin2_rad: F32
    telemetry fin3_rad: F32
    telemetry fin4_rad: F32
    telemetry satCount: U32

    # ------------------------------------------------------------ 事件
    event ClosedLoopOn() severity activity high format "Controller 進入閉環"
    event ClosedLoopOff(reason: CtlHoldReason) severity activity high format "Controller 離開閉環 reason={}"

    # ------------------------------------------------------------ 參數（SSS FlightParams）
    param kp: F32 default 1.2
    param kd: F32 default 0.25
    @ ±30° = 0.5235988 rad
    param finMax_rad: F32 default 0.5235988
    param stateStale_s: F32 default 0.020
    @ A9 Demo-1-1（50 m 固定翼）：主動段舵恆 0
    param demo50mFixed: bool default false
  }

}
