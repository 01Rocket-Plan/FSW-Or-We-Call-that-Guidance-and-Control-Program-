# Host fpp-check stubs until lib/fprime is vendored (MCU group).
# Minimal Fw/Svc surface needed by PadLink／PadController／UmbilicalLink.

module Fw {

  port Time
  port Tlm
  port Log
  port LogText
  port PrmGet
  port PrmSet
  port Cmd
  port CmdReg
  port CmdResponse

  type Buffer

  port BufferSend(
    ref fwBuffer: Buffer
  )

}

module Svc {

  port Sched(
    context: U32
  )

}
