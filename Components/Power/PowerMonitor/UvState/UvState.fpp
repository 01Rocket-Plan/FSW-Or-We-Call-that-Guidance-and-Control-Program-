module Components {

    @ Under-voltage state machine (lives under PowerMonitor/; Base ID 0x6200).
    @ Emits L1/L2 load-shed requests only. Does NOT command actuators / L3.
    @ Uses RocketTypes.UvState; SEP and chute channels are never shed.
    passive component UvStateComp {

        @ Power rail telemetry sample input.
        sync input port powerIn: Components.PowerRailTlmPort

        @ Optional scheduler tick; time is injected each tick.
        sync input port schedIn: Svc.Sched

        @ Current under-voltage level for flightMode / actuatorMgr.
        output port uvStateOut: RocketTypes.UvStatePort

        @ L1 load-shed request: camera, gimbal, humidity sensor off.
        output port shedL1: Fw.Signal

        @ L2 load-shed request: high-rate logger off, reduce tlm rate.
        output port shedL2: Fw.Signal

        telemetry level: RocketTypes.UvState
        telemetry dwellAccum_s: F32
        telemetry rejectedCount: U32
        telemetry transitionCount: U32

        event levelChanged(old: RocketTypes.UvState, new: RocketTypes.UvState) \
            severity activity high \
            format "UV level changed {} -> {}"

        event shedL1Requested(level: RocketTypes.UvState) \
            severity warning high \
            format "L1 load-shed requested at level {}"

        event shedL2Requested(level: RocketTypes.UvState) \
            severity warning high \
            format "L2 load-shed requested at level {}"

        event sampleRejected(rail_id: U32) \
            severity warning low \
            format "Rejected power sample on rail {}"

        # Parameters (placeholders; real values from prmDb).
        param warnEntry_V: F32 default 0.0
        param shedEntry_V: F32 default 0.0
        param criticalEntry_V: F32 default 0.0
        param warnDwell_s: F32 default 0.0
        param shedDwell_s: F32 default 0.0
        param criticalDwell_s: F32 default 0.0
        param warnHyst_V: F32 default 0.0
        param shedHyst_V: F32 default 0.0
        param criticalHyst_V: F32 default 0.0
        param maxSampleAge_s: F32 default 0.0

        sync command RESET()
        sync command RELOAD_CONFIG()

        time get port timeCaller
        command recv port cmdIn
        command reg port cmdRegOut
        command resp port cmdResponseOut
        event port eventOut
        telemetry port tlmOut
        text event port textEventOut
        param get port prmGetOut
        param set port prmSetOut
    }
}
