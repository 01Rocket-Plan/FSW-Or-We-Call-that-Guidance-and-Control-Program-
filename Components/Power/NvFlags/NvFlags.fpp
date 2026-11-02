module Components {
    @ Non-volatile flag store with dual-slot ping-pong and CRC.
    @ Base ID 0x6100 per docs/conventions.md (Sim = 0x6180).
    @ Port types live in module Power (see NvFlagsPorts.fpp) so peers can
    @ declare matching output ports and wire them in the topology.
    passive component NvFlags {

        @ Guarded set of flag bits 0..3; OR into current mask and commit.
        guarded input port sepReleasedSet: Power.SepReleasedSet

        @ Synchronous read of RAM-cached mask loaded from NVS at init.
        sync input port sepReleasedGet: Power.SepReleasedGet

        @ Set-only latch: 分離已下令 (sepCommanded).
        guarded input port sepCommanded: Power.SepCommanded

        @ Set-only latch: DS 傘已下令 (dsChuteCommanded).
        guarded input port dsChuteCommanded: Power.DsChuteCommanded

        @ Get current phase.
        sync input port phaseGet: Power.PhaseGet

        @ Set phase (persisted via dual-slot + CRC; skip if identical).
        guarded input port phaseSet: Power.PhaseSet

        time get port timeCaller

        telemetry FlagsMask: U8 update on change
        telemetry Phase: U32 update on change
        telemetry WriteCount: U32 update on change
        telemetry SepCommanded: bool update on change
        telemetry DsChuteCommanded: bool update on change

        event NvSepLatched(mask: U8) severity activity high \
            format "sepReleased latched: {}"
        event NvWriteFail(mask: U8) severity warning high \
            format "NVS write failed for mask {}"

        command recv port cmdIn
        command reg port cmdRegOut
        command resp port cmdResponseOut
        event port eventOut
        text event port textEventOut
        telemetry port tlmOut
    }
}
