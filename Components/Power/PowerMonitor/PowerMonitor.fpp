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

    @ PowerMonitor rail sampler (Base ID 0x6000 per docs/conventions.md; Sim = 0x6080).
    @ Samples three power rails on a 100 Hz rate group.
    active component PowerMonitor {

        # ------------------------------------------------------------------
        # Standard ports
        # ------------------------------------------------------------------
        time get port timeCaller
        telemetry port tlmOut
        event port eventOut
        text event port textEventOut
        command recv port cmdIn
        command reg port cmdRegOut
        command resp port cmdResponseOut
        param get port prmGetOut
        param set port prmSetOut

        # ------------------------------------------------------------------
        # Scheduling
        # ------------------------------------------------------------------
        @ 100 Hz rate group tick (rg100Hz).
        sync input port schedIn: Svc.Sched

        @ Latest rail sample output for UvState / consumers.
        output port railOut: Components.PowerRailTlmPort

        # ------------------------------------------------------------------
        # Telemetry
        # ------------------------------------------------------------------
        @ Per-rail telemetry sample (emitted once per rail per tick).
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
