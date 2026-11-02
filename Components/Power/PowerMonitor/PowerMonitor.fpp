module Components {

    enum PowerRailId {
        BATT_MAIN  = 0
        SERVO_5V   = 1
        LOGIC_3V3  = 2
    }

    struct PowerRailTlm {
        rail_id: PowerRailId
        voltage_V: F32
        current_A: F32
        sample_age_s: F32
        valid: bool
    }

    port PowerRailTlmPort(sample: PowerRailTlm)

    @ 電壓監測：電源軌取樣器（Base ID 0x6000／docs/conventions.md；Sim = 0x6080）。
    @ 在 100 Hz rate group 上取樣三條電源軌（BATT_MAIN／SERVO_5V／LOGIC_3V3）。
    @ 本元件只管電壓遙測；供電路徑（誰供電給誰）由硬體配線／拓撲決定，不在此軟體範圍。
    active component PowerMonitor {

        # ------------------------------------------------------------------
        # Standard ports
        # ------------------------------------------------------------------
        time get port timeCaller
        telemetry port tlmOut
        event port eventOut
        text event port textEventOut
        @ 指令分發器（Svc.CommandDispatcher / cmdDisp）
        command recv port cmdIn
        @ 指令分發器 — 註冊
        command reg port cmdRegOut
        @ 指令分發器 — 回應
        command resp port cmdResponseOut
        param get port prmGetOut
        param set port prmSetOut

        # ------------------------------------------------------------------
        # Scheduling
        # ------------------------------------------------------------------
        @ 100 Hz rate group tick（rg100Hz）
        sync input port schedIn: Svc.Sched

        @ 電壓監測：最新軌樣本輸出 → UvState／消費者
        output port railOut: Components.PowerRailTlmPort

        # ------------------------------------------------------------------
        # Telemetry
        # ------------------------------------------------------------------
        @ 每軌遙測樣本（每 tick 每軌發一次）
        telemetry PowerRailTlm: PowerRailTlm

        # ------------------------------------------------------------------
        # Events
        # ------------------------------------------------------------------
        event RailReadFail(rail: PowerRailId) \
            severity warning high \
            format "Power rail {} read failed"

        event RailFrozen(rail: PowerRailId) \
            severity warning high \
            format "Power rail {} appears frozen"

        # ------------------------------------------------------------------
        # Parameters (divider ratio / offset in prmDb)
        # ------------------------------------------------------------------
        param VREF_V: F32 default 3.3
        param FULL_SCALE: U32 default 4095
        param DIVIDER_BATT_MAIN: F32 default 1.0
        param DIVIDER_SERVO_5V: F32 default 1.0
        param DIVIDER_LOGIC_3V3: F32 default 1.0
        param CAL_OFFSET_BATT_MAIN: F32 default 0.0
        param CAL_OFFSET_SERVO_5V: F32 default 0.0
        param CAL_OFFSET_LOGIC_3V3: F32 default 0.0
        param FROZEN_THRESHOLD_S: F32 default 1.0

        # ------------------------------------------------------------------
        # Commands
        # ------------------------------------------------------------------
        async command SET_SIM(enabled: bool)
        async command SIM_READ_FAIL(rail: PowerRailId, fail: bool)
        async command SIM_FROZEN(rail: PowerRailId, frozen: bool)
    }
}
