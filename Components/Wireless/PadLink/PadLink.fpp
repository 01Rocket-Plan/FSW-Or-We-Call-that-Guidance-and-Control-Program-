module Components {

  @ 發射架主控狀態（數值與 PadControllerCore.hpp 的 PadLink::PadState 一致）
  enum PadState : U8 {
    SAFE = 0
    ARMED = 1
    COUNTDOWN = 2
    IGNITION = 3
    RELEASED = 4
    LIFTED_OFF = 5
    HANG_FIRE = 6
    HOLD_ABORT = 7
    NO_LIFTOFF = 8
  }

  @ 退回／拒絕原因（數值與 PadLink::PadReason 一致）
  enum PadReason : U8 {
    NONE = 0
    CMD_ABORT = 1
    CMD_DISARM = 2
    NOT_SEATED = 3
    NO_CONTINUITY = 4
    KEY_OFF = 5
    TILT = 6
    RAIL_JAM = 7
    PAD_POWER = 8
    GROUND_LINK_LOST = 9
    ROCKET_LINK_LOST = 10
    ROCKET_NOT_READY = 11
    NO_THRUST = 12
    NO_LIFTOFF = 13
    IGNITION_LOCKED = 14
    NOT_ARMED = 15
    BUSY = 16
  }

  @ 地面 → 架指令碼（數值與 PadLink::PadCmdOp 一致）
  enum PadCmdOp : U8 {
    ARM = 1
    LAUNCH = 2
    ABORT = 3
    DISARM = 4
    HEARTBEAT = 5
  }

  @ 指令處理結果（數值與 PadLink::PadCmdResult 一致）
  enum PadCmdResult : U8 {
    NONE = 0
    ACCEPTED = 1
    AUTH_FAIL = 2
    NO_VERIFIER = 3
    REPLAY = 4
    WRONG_BOOT = 5
    BAD_FRAME = 6
    REJECTED = 7
  }

  @ 架上硬體讀值（PadMcu GPIO／ADC 驅動每拍提供；SI 單位）
  struct PadIoSample {
    seated: bool
    continuity: bool
    keyOn: bool
    railJam: bool
    offRail: bool
    thrustOk: bool
    tilt_rad: F32
    padV_V: F32
  }

  @ 架上致動命令：點火繼電器（軟體這一道，實體鑰匙在硬體串聯）與夾持
  struct PadActCmd {
    igniterOn: bool
    clampRelease: bool
  }

  @ 讀架上硬體（同步呼叫，回傳本拍讀值）
  port PadIoGet -> PadIoSample

  @ 送架上致動命令
  port PadActPort(cmd: PadActCmd)

  @ 箭體 BIT 旗標（位元定義見 PadLinkCodec.hpp 的 RocketBitFlag）
  port RocketBitPort(bitFlags: U16, battV_V: F32)

}
