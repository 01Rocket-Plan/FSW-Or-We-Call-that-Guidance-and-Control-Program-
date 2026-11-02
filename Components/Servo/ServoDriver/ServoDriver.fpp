# 元件指定：型號 SG90 微型伺服。
# 用途（共用同一元件定義、拓樸中為兩個獨立實例）：
#  (a) DsMcu 尾翼 TAIL_1–4：Base ID 0x3000，channel_count=4，GPIO 25/26/27/14，LEDC 高速 ch0–3、timer0。
#  (b) UsMcu 相機二軸雲台 PITCH/ROLL：實例 gimbalServoDriver，Base ID 0x3200，channel_count=2，GPIO 25(PITCH)/26(ROLL)，LEDC 高速 ch0–1。
# 訊號：50 Hz PWM，脈寬參數夾限 1000–2000 µs，中立 1500 µs。
# 腳位為 MCU 組 v0.1 草案，待 Anatoly 確認，實際值以 Components/Servo/Pwm/ServoPwmPins.hpp 為準。
# 供電：供電依電力組定案（待確認）。
#
module Components {

  @ SG90 尾翼（DS）／雲台（US 實例）PWM 驅動。Base ID 0x3000；雲台實例 0x3200。
  @ 接 rg1kHz；內部依 pwm_hz（預設 50）零階保持。µs 硬限 pulse_min/max。不 sleep、時間只取自 F´ 時間服務。
  passive component ServoDriver {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    @ actuatorMgr → 本驅動（合約 cmdIn）
    sync input port cmdIn: RocketTypes.ServoCmdPort
    @ 欠壓狀態：只記事件，不動舵（E6）
    sync input port uvIn: RocketTypes.UvStatePort

    @ 實際輸出 µs（Sim 拓樸可接；真機可懸空）
    output port pwmOut: RocketTypes.ServoCmdPort

    command recv port cmdDispIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry drvState: RocketTypes.DrvState
    telemetry lastSeq: U32
    telemetry lastPulse0_us: U16
    telemetry lastPulse1_us: U16
    telemetry lastPulse2_us: U16
    telemetry lastPulse3_us: U16
    telemetry cmdAge_s: F32
    telemetry uvState: RocketTypes.UvState
    telemetry clampCount: U32
    telemetry dropCount: U32

    event EnterReady() severity activity high format "ServoDriver READY"
    event EnterFault(reason: U8) severity warning high format "ServoDriver FAULT reason={}"
    event CmdDropped(seq: U32, reason: U8) severity warning low format "ServoCmd drop seq={} reason={}"
    event PulseClamped(ch: U8, raw: U16, used: U16) severity warning low format "ch{} clamp {}→{}"
    event CmdTimeout(age_s: F32) severity warning high format "ServoCmd timeout age_s={}"
    event LockNeutral(source: U8) severity activity high format "lockNeutral source={}"
    event UvSeen(st: RocketTypes.UvState) severity activity high format "UvState={}"

    param neutral_us: U16 default 1500
    param pulse_min_us: U16 default 1000
    param pulse_max_us: U16 default 2000
    param cmd_timeout_s: F32 default 0.05
    param pwm_hz: U16 default 50
    param channel_count: U8 default 4
    param selftest_deadline_s: F32 default 1.0
  }

}
