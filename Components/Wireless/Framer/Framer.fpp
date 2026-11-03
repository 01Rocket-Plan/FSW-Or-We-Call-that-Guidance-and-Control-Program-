module Components {

  @ Framer：§2 空中幀組幀；Base ID 0x8100；被動元件
  @ frameMax_B 預設 0＝未設定（TBD／待 Edward 定射頻 MTU）
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

    @ 單幀上限（bytes）；0＝未設定（TBD）
    param frameMax_B: U32 default 0

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
