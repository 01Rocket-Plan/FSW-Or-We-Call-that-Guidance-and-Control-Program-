module Mcu {

  @ 重開機原因；數值對齊 PlatformCore.hpp ResetReason
  enum ResetReason {
    UNKNOWN = 0
    POWER_ON = 1
    SOFTWARE = 2
    WATCHDOG = 3
    BROWNOUT = 4
    PANIC = 5
  }

  @ 平台：主迴路踢狗、1 kHz 迴圈間隔監測、重開機原因。
  @ Base ID 0x1000（docs/conventions.md §5）。**必須是 rg1kHz 的最後一個成員**：
  @ 鏈上任何元件卡住 → 這裡不會被叫 → 不踢狗 → 硬體看門狗重開（ESP32）。
  @ 硬體差異在 IPlatformHal；SIL 用 LinuxPlatformHal（不碰硬體）。
  passive component Platform {

    @ rg1kHz（最後一個）
    guarded input port schedIn: Svc.Sched

    time get port timeCaller
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry resetReason: ResetReason
    telemetry ticks: U32
    telemetry overruns: U32
    telemetry maxLoopDt_s: F32

    event Boot(reason: ResetReason) severity activity high format "Platform 開機 reset={}"
    event AbnormalReset(reason: ResetReason) severity warning high \
      format "Platform 上次是異常重開（{}）：FlightMode 應進安全態（FM-20）"
    event LoopOverrun(dt_s: F32) severity warning low format "Platform 1 kHz 迴圈超時 dt_s={}"

    @ 清掉 maxLoopDt_s
    sync command CLEAR_MAX()

    param loopPeriod_s: F32 default 0.001
    param overrun_s: F32 default 0.0015
  }

}
