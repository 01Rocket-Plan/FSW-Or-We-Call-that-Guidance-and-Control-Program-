@ SdLogger — DS/US flight recorder (紀錄組).
@ Component base ID: 0x2000 (docs/conventions.md §5); Sim instance base = 0x2080.
@ Channel/event local opcodes below are relative to that base; absolute = base + local.
@ Logging rules (合約 §9 LOGGING): channel seq continuous, timestamps monotonic,
@ queue full → drop + event, never block. Time only from F´ time service (no sleep).
module Components {

  @ SD card / filesystem status for recorder health telemetry
  enum SdCardStatus { OK, INIT, BUSY, CARD_MISSING, IO_ERROR, FULL }

  @ DS/US SD logger. Owns one record queue and one SD write buffer pool; both have HWM channels.
  @ Data-in ports and media driver are implemented by 紀錄組子Bot2; this FPP fixes the dictionary.
  passive component SdLogger {

    @ Rate-group tick (e.g. rg1Hz / rg10Hz) for flush / HWM publish — topology decides
    sync input port schedIn: Svc.Sched

    @ F´ time service only (LOGGING timestamps)
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
    # Telemetry — local IDs 0x00.. (absolute = 0x2000 + local)
    # ------------------------------------------------------------------

    @ local 0x00 / abs 0x2000 — records successfully written to media
    telemetry WriteCount: U32

    @ local 0x01 / abs 0x2001 — records dropped (queue full or policy)
    telemetry DropCount: U32

    @ local 0x02 / abs 0x2002 — high-water mark of the inbound record queue (depth)
    telemetry RecordQueueHwm: U32

    @ local 0x03 / abs 0x2003 — high-water mark of the SD write buffer pool (buffers in use)
    telemetry SdWritePoolHwm: U32

    @ local 0x04 / abs 0x2004 — cumulative SD / media errors
    telemetry SdErrorCount: U32

    @ local 0x05 / abs 0x2005 — current SD / media status
    telemetry SdStatus: SdCardStatus

    @ local 0x06 / abs 0x2006 — last accepted channel seq (continuity monitor)
    telemetry LastSeq: U32

    @ local 0x07 / abs 0x2007 — last accepted sample timestamp (s, F´ time)
    telemetry LastT_s: F64

    # ------------------------------------------------------------------
    # Events — local IDs 0x00.. (absolute = 0x2000 + local; topology may remap)
    # ------------------------------------------------------------------

    @ local 0x00 / abs 0x2000 — queue full: drop and continue (never block)
    event QueueFullDrop(dropped: U32, queueDepth: U32) \
      severity warning high \
      format "SdLogger queue full drop={} depth={}"

    @ local 0x01 / abs 0x2001 — media / SD error
    event SdError(status: SdCardStatus, code: U8) \
      severity warning high \
      format "SdLogger SD error status={} code={}"

    @ local 0x02 / abs 0x2002 — logger READY
    event EnterReady() \
      severity activity high \
      format "SdLogger READY"

    @ local 0x03 / abs 0x2003 — logger FAULT
    event EnterFault(reason: U8) \
      severity warning high \
      format "SdLogger FAULT reason={}"

    @ Max depth of the inbound record queue (HWM ceiling)
    param RECORD_QUEUE_DEPTH: U32 default 64
    @ SD write buffer pool size (buffers)
    param SD_WRITE_POOL_SIZE: U32 default 8
  }

}
