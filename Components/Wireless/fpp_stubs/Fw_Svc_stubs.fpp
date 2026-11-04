# Host fpp-check stubs for Components/Wireless.

module Fw {
  enum Success { FAILURE, SUCCESS }
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
  port BufferSend(ref fwBuffer: Buffer)
}

module Svc {
  port Sched(context: U32)
  port ComDataWithContext(ref data: Fw.Buffer, context: U32)
  enum ComStatusKind { READY, BUSY }
  port ComStatus(status: ComStatusKind)
}
