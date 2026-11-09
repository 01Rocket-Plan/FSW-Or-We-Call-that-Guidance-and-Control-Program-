module Mcu {

  @ ActuatorMgr 為何鎖中位（遙測）；數值對齊 ActuatorMgrCore.hpp LockReason
  enum ActLockReason {
    NONE = 0
    NO_CMD = 1
    CMD_LOCK = 2
    CMD_STALE = 3
    UNDERVOLT = 4
  }

  @ DS 作動管理 W4：FinCmd(rad) → ServoCmd(µs)；SepCmd 轉發（可錯開）
  @ Base ID 0x1300（docs/conventions.md §5）。接 rg1kHz，排在 controller 之後、servoDriver 之前。
  @ µs 只從這裡開始出現（conventions §3）。欠壓鎖中位由這裡經 lockNeutral 下令（E6）；
  @ 欠壓不擋分離（gates：分離不鎖）。邏輯全在 ActuatorMgrCore（純 C++）。
  passive component ActuatorMgr {

    @ rg1kHz
    guarded input port schedIn: Svc.Sched

    @ controller → 四片尾翼角（rad）
    guarded input port finCmdIn: RocketTypes.FinCmdPort

    @ flightMode → 分離指令
    guarded input port sepCmdIn: RocketTypes.SepCmdPort

    @ powerMonitor／UvState → 欠壓等級
    guarded input port uvIn: RocketTypes.UvStatePort

    @ → servoDriver（TAIL，0x3000）
    output port servoCmdOut: RocketTypes.ServoCmdPort

    @ → sepServoDriver（SEP，0x3100）
    output port sepCmdOut: RocketTypes.SepCmdPort

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
    telemetry locked: bool
    telemetry lockReason: ActLockReason
    telemetry pulse1_us: U16
    telemetry pulse2_us: U16
    telemetry pulse3_us: U16
    telemetry pulse4_us: U16
    telemetry clampCount: U32
    telemetry rateLimitCount: U32
    telemetry uvState: RocketTypes.UvState
    telemetry sepPendingMask: U8
    telemetry sepDropCount: U32

    # ------------------------------------------------------------ 事件
    event Locked(reason: ActLockReason) severity activity high format "ActuatorMgr 尾翼鎖中位 reason={}"
    event Unlocked() severity activity high format "ActuatorMgr 尾翼解鎖"
    event SepForwarded(mask: U8) severity activity high format "ActuatorMgr 分離轉發 mask=0x{x}"
    event SepDropped(seq: U32) severity warning low format "ActuatorMgr SepCmd 丟棄 seq={}（valid=false 或 seq 重複）"

    # ------------------------------------------------------------ 參數
    @ SG90 中位／硬限（Components/Servo/ServoTypes）
    param neutral_us: U16 default 1500
    param pulseMin_us: U16 default 1000
    param pulseMax_us: U16 default 2000
    @ SG90：1000 µs / 180°（Sg90Core kTravelDeg 假設，未實測）
    param usPerRad: F32 default 318.30989
    @ 舵機角 / 尾翼角（連桿比）
    param gear: F32 default 1.0
    @ 每通道正反向 ±1（冷測定）
    param dir1: F32 default 1.0
    param dir2: F32 default 1.0
    param dir3: F32 default 1.0
    param dir4: F32 default 1.0
    @ 每通道機械中位配平（舵機角 rad）
    param trim1_rad: F32 default 0.0
    param trim2_rad: F32 default 0.0
    param trim3_rad: F32 default 0.0
    param trim4_rad: F32 default 0.0
    @ ±30°
    param finMax_rad: F32 default 0.5235988
    @ SG90 約 0.1 s/60°
    param rateMax_rads: F32 default 10.47
    param finStale_s: F32 default 0.020
    @ ≥ 此等級鎖中位
    param uvLockLevel: RocketTypes.UvState default RocketTypes.UvState.UV_SHED
    @ 分離通道錯開；0 = 同時（SSS「錯開 Δ ms」Δ 未定）
    param sepStagger_s: F32 default 0.0
  }

}
