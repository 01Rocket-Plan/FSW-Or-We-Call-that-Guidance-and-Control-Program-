module Components {

  @ Framer：§2 空中幀組幀；Base ID 0x8100；被動元件
  @ 射頻型號：LoRa Ra-02（433 MHz），晶片 Semtech SX1278
  @ frameMax_B 預設 255＝SX1278 單包酬載上限（FIFO 256 B，但 RegPayloadLength 為 8 bit，最大 255）
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

    @ 單幀上限（bytes）；預設 255＝LoRa Ra-02／SX1278 單包上限
    param frameMax_B: U32 default 255

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
