module Mcu {

  @ SIL 專用：每拍送一筆「墊上靜置」的 ImuSample（機體系比力 (g·cosθ, 0, g·sinθ)（機體 X 指鼻；θ=0 時 (+g,0,0)）、角速度 0）。
  @ Base ID 0x1080（Platform＋0x80，Sim 慣例）。
  @ 只是佔位：EnvSensing 的 ImuDriverSim 用真 F´ 編譯過後，DsMcu SIL 拓樸改接它，本元件刪除。
  passive component SilImuFeed {

    guarded input port schedIn: Svc.Sched
    output port imuOut: RocketTypes.ImuData

    time get port timeCaller
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry seq: U32

    @ 開機後多久開始送 valid=true（模擬驅動 SELFTEST）
    param readyDelay_s: F32 default 0.1
    param g_mps2: F32 default 9.80665
    @ 導軌傾角（繞機體 Y 俯仰）
    param railTilt_rad: F32 default 0.0
  }

}
