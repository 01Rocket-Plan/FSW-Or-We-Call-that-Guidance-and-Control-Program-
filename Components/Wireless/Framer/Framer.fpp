module Components {

  @ Framer：§2 空中幀組幀；Base ID 0x8100；被動元件
  @ frameMax_B 預設 256＝對齊 LoRa-02（SX1278-class）256 Byte buffer（Edward）
  passive component Framer {

    sync input port comIn: Svc.ComDataWithContext
    output port framedOut: Svc.ComDataWithContext
    output port bufferAllocate: Fw.BufferSend
    output port bufferDeallocate: Fw.BufferSend

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 單幀上限（bytes）；預設 256＝LoRa-02 buffer
    param frameMax_B: U32 default 256

    telemetry FramesBuilt: U32
    telemetry LastFrameLen: U32
    telemetry RejectCount: U32

    event FramerError(code: U8, detail: U32) \
      severity warning low \
      format "Framer error code={} detail={}"

    event FramerBuilt(frameType: U8, apid: U16, seq: U32, len: U32) \
      severity activity low \
      format "Framer built frameType={} apid={} seq={} len={}"
  }

}
