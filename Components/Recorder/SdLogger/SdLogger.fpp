@ SdLogger — DS/US flight recorder (紀錄組).
@ Component base ID: 0x2000 (docs/conventions.md §5); Sim instance base = 0x2080.
@ Channel/event local opcodes below are relative to that base; absolute = base + local.
@ Logging rules (合約 §9 LOGGING): channel seq continuous, timestamps monotonic,
@ queue full → drop newest + event, never block. Time only from F´ time service (no sleep).
@ Media card (Edward／首席): Micro SDHC UHS-I 16GB. Card reader module model: TBD.
module Components {

  @ Removable media card model (not the SPI/SD reader board — that remains TBD)
  enum SdCardModel {
    MICRO_SDHC_UHS_I_16GB
  }

  @ DS vs US logger instance role
  enum SdCardRole { DS_LOG, US_LOG }

  @ SD card / filesystem status for recorder health telemetry
  enum SdCardStatus { OK, INIT, BUSY, CARD_MISSING, IO_ERROR, FULL }

  @ Sim-only media fault injection
  enum SdSimFault { NONE, FULL, NACK, WRITE_FAIL, CARD_MISSING }

  @ DS/US SD logger. Owns one record queue and one SD write buffer pool; both have HWM channels.
  @ Dictionary locals 0x00–0x07 / events 0x00–0x05 owned by 子Bot1; Bot2 TLM 0x08–0x0C; image EVR from 0x06 / TLM from 0x0D.
  passive component SdLogger {

    @ Rate-group tick (e.g. rg1Hz / rg10Hz) for flush / HWM publish — topology decides
    sync input port schedIn: Svc.Sched

    @ DS / shared sample inputs (record-only; unused ports stay unwired per deployment)
    sync input port imuIn: RocketTypes.ImuData
    sync input port baroIn: RocketTypes.BaroData
    sync input port envIn: RocketTypes.EnvData
    sync input port gnssIn: RocketTypes.GnssData
    sync input port stateIn: RocketTypes.StateData
    sync input port finCmdIn: RocketTypes.FinCmdPort
    sync input port servoCmdIn: RocketTypes.ServoCmdPort
    sync input port sepCmdIn: RocketTypes.SepCmdPort

    @ US sample inputs (record-only layers; no I2C/SPI from Recorder)
    sync input port mpu6050In: RocketTypes.Mpu6050Data
    sync input port humTempIn: RocketTypes.HumTempData

    @ US_CAM 2KB image chunks (Ov7670 imageOut → sdLogger)；埠型別在 Components/Camera/CamTypes
    sync input port usImageIn: CamImageOut
    @ DS_CAM 2KB image chunks (Esp32Cam imageOut → sdLogger)
    sync input port dsImageIn: CamImageOut

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

    @ Start accepting / writing records
    sync command START
    @ Stop accepting new records (flush may still run on schedIn)
    sync command STOP
    @ Best-effort flush of queued records to media
    sync command FLUSH

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

    @ local 0x06 / abs 0x2006 — last accepted upstream SampleHeader.seq (continuity monitor)
    telemetry LastSeq: U32

    @ local 0x07 / abs 0x2007 — last accepted sample timestamp (s, F´ time)
    telemetry LastT_s: F64

    @ local 0x08 / abs 0x2008 — logger handshake (OFF/INIT/SELFTEST/READY/FAULT)
    telemetry HsState: RocketTypes.DrvState

    @ local 0x09 / abs 0x2009 — last logger-owned contiguous write sequence
    telemetry LoggerSeq: U32

    @ local 0x0A / abs 0x200A — last producer SampleHeader.seq stored with a write
    telemetry ProducerSeq: U32

    @ local 0x0B / abs 0x200B — configured media card model
    telemetry CardModel: SdCardModel

    @ local 0x0C / abs 0x200C — configured DS/US role
    telemetry CardRole: SdCardRole

    @ local 0x0D / abs 0x200D — image chunks successfully written
    telemetry ImageChunkCount: U32

    @ local 0x0E / abs 0x200E — image chunks dropped (image queue full)
    telemetry ImageDropCount: U32

    @ local 0x0F / abs 0x200F — image frames completed (END seen)
    telemetry ImageFrameCount: U32

    @ local 0x10 / abs 0x2010 — cumulative image payload bytes written
    telemetry ImageBytesWritten: U32

    @ local 0x11 / abs 0x2011 — image chunk queue high-water mark
    telemetry ImageQueueHwm: U32

    @ local 0x12 / abs 0x2012 — last accepted image frameNumber
    telemetry LastImageFrame: U32

    @ local 0x13 / abs 0x2013 — last accepted image chunkIndex
    telemetry LastImageChunk: U32

    # ------------------------------------------------------------------
    # Events — local IDs 0x00.. (absolute = 0x2000 + local; topology may remap)
    # ------------------------------------------------------------------

    @ local 0x00 / abs 0x2000 — queue full: drop newest and continue (never block)
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

    @ local 0x04 / abs 0x2004 — upstream channel seq discontinuity (expected vs got)
    event SeqGap(expected: U32, got: U32) \
      severity warning high \
      format "SdLogger seq gap expected={} got={}"

    @ local 0x05 / abs 0x2005 — sample timestamp went backwards (prev vs now, seconds)
    event TimeNonMonotonic(prev_s: F64, now_s: F64) \
      severity warning high \
      format "SdLogger time non-monotonic prev_s={} now_s={}"

    @ local 0x06 / abs 0x2006 — image queue full: drop newest chunk (never block)
    event ImageChunkDrop(dropped: U32, queueDepth: U32) \
      severity warning high \
      format "SdLogger image chunk drop={} depth={}"

    @ local 0x07 / abs 0x2007 — image frameNumber discontinuity
    event ImageFrameGap(expected: U32, got: U32) \
      severity warning high \
      format "SdLogger image frame gap expected={} got={}"

    @ local 0x08 / abs 0x2008 — image chunkIndex discontinuity within a frame
    event ImageChunkGap(frameNumber: U32, expected: U32, got: U32) \
      severity warning high \
      format "SdLogger image chunk gap frame={} expected={} got={}"

    @ local 0x09 / abs 0x2009 — accepted chunk carried CamChunkFlag.TRUNCATED
    event ImageTruncatedSeen(frameNumber: U32, chunkIndex: U32) \
      severity warning high \
      format "SdLogger image truncated seen frame={} chunk={}"

    @ Max depth of the inbound record queue (HWM ceiling)
    param RECORD_QUEUE_DEPTH: U32 default 64
    @ SD write buffer pool size (buffers)
    param SD_WRITE_POOL_SIZE: U32 default 8
    @ Media card model (Micro SDHC UHS-I 16GB)
    param CARD_MODEL: SdCardModel default SdCardModel.MICRO_SDHC_UHS_I_16GB
    @ Instance role DS_LOG or US_LOG
    param CARD_ROLE: SdCardRole default SdCardRole.DS_LOG
    @ Max depth of the inbound image-chunk queue (2KB entries)
    param IMAGE_QUEUE_DEPTH: U32 default 8
  }

  @ SdLogger Sim：同一 SdLoggerCore + SdCardSimDevice（base id 0x2080）
  passive component SdLoggerSim {

    sync input port schedIn: Svc.Sched

    sync input port imuIn: RocketTypes.ImuData
    sync input port baroIn: RocketTypes.BaroData
    sync input port envIn: RocketTypes.EnvData
    sync input port gnssIn: RocketTypes.GnssData
    sync input port stateIn: RocketTypes.StateData
    sync input port finCmdIn: RocketTypes.FinCmdPort
    sync input port servoCmdIn: RocketTypes.ServoCmdPort
    sync input port sepCmdIn: RocketTypes.SepCmdPort
    sync input port mpu6050In: RocketTypes.Mpu6050Data
    sync input port humTempIn: RocketTypes.HumTempData

    sync input port usImageIn: CamImageOut
    sync input port dsImageIn: CamImageOut

    time get port timeCaller
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    param get port prmGetOut
    param set port prmSetOut

    sync command START
    sync command STOP
    sync command FLUSH
    @ Inject media fault into SdCardSimDevice
    sync command SIM_INJECT(fault: SdSimFault)

    telemetry WriteCount: U32
    telemetry DropCount: U32
    telemetry RecordQueueHwm: U32
    telemetry SdWritePoolHwm: U32
    telemetry SdErrorCount: U32
    telemetry SdStatus: SdCardStatus
    telemetry LastSeq: U32
    telemetry LastT_s: F64
    telemetry HsState: RocketTypes.DrvState
    telemetry LoggerSeq: U32
    telemetry ProducerSeq: U32
    telemetry CardModel: SdCardModel
    telemetry CardRole: SdCardRole
    telemetry ImageChunkCount: U32
    telemetry ImageDropCount: U32
    telemetry ImageFrameCount: U32
    telemetry ImageBytesWritten: U32
    telemetry ImageQueueHwm: U32
    telemetry LastImageFrame: U32
    telemetry LastImageChunk: U32

    event QueueFullDrop(dropped: U32, queueDepth: U32) \
      severity warning high \
      format "SdLoggerSim queue full drop={} depth={}"

    event SdError(status: SdCardStatus, code: U8) \
      severity warning high \
      format "SdLoggerSim SD error status={} code={}"

    event EnterReady() \
      severity activity high \
      format "SdLoggerSim READY"

    event EnterFault(reason: U8) \
      severity warning high \
      format "SdLoggerSim FAULT reason={}"

    event SeqGap(expected: U32, got: U32) \
      severity warning high \
      format "SdLoggerSim seq gap expected={} got={}"

    event TimeNonMonotonic(prev_s: F64, now_s: F64) \
      severity warning high \
      format "SdLoggerSim time non-monotonic prev_s={} now_s={}"

    event ImageChunkDrop(dropped: U32, queueDepth: U32) \
      severity warning high \
      format "SdLoggerSim image chunk drop={} depth={}"

    event ImageFrameGap(expected: U32, got: U32) \
      severity warning high \
      format "SdLoggerSim image frame gap expected={} got={}"

    event ImageChunkGap(frameNumber: U32, expected: U32, got: U32) \
      severity warning high \
      format "SdLoggerSim image chunk gap frame={} expected={} got={}"

    event ImageTruncatedSeen(frameNumber: U32, chunkIndex: U32) \
      severity warning high \
      format "SdLoggerSim image truncated seen frame={} chunk={}"

    param RECORD_QUEUE_DEPTH: U32 default 64
    param SD_WRITE_POOL_SIZE: U32 default 8
    param CARD_MODEL: SdCardModel default SdCardModel.MICRO_SDHC_UHS_I_16GB
    param CARD_ROLE: SdCardRole default SdCardRole.DS_LOG
    param IMAGE_QUEUE_DEPTH: U32 default 8
  }

}
