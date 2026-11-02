module Components {

    @ 電壓監測／欠壓狀態機（位於 PowerMonitor/；Base ID 0x6200）。
    @ 只發出 L1／L2 卸載請求，不下令致動器／L3。
    @ 使用 RocketTypes.UvState；SEP 與傘通道永不卸載。
    passive component UvStateComp {

        @ 電壓監測：電源軌遙測樣本輸入（PowerRailTlm → 欠壓分級）
        sync input port powerIn: Components.PowerRailTlmPort

        @ 可選排程 tick；每次 tick 注入時間
        sync input port schedIn: Svc.Sched

        @ 目前欠壓等級 → flightMode／actuatorMgr
        output port uvStateOut: RocketTypes.UvStatePort

        @ L1 卸載請求：相機、雲台、濕度感測關閉
        output port shedL1: Fw.Signal

        @ L2 卸載請求：高速 logger 關閉、降低遙測率
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
