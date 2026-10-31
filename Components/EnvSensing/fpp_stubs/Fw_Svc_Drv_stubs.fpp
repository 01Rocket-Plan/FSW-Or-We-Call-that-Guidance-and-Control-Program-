# Host fpp-check stubs until lib/fprime is vendored (MCU group).
# Special port names match F´ conventions (Fw.Time, Fw.LogText, …).

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

  struct Buffer {
    buffSize: U32
  }

}

module Svc {

  port Sched(
    context: U32
  )

}

module Drv {

  enum I2cStatus {
    I2C_OK
    I2C_ADDRESS_ERR
    I2C_WRITE_ERR
    I2C_READ_ERR
    OTHER_ERR
  }

  enum SpiStatus {
    SPI_OK
    SPI_WRITE_ERR
    SPI_READ_ERR
    OTHER_ERR
  }

  port I2c(
    addr: U32
    writeBuffer: Fw.Buffer
    readBuffer: Fw.Buffer
  ) -> I2cStatus

  port Spi(
    writeBuffer: Fw.Buffer
    readBuffer: Fw.Buffer
  ) -> SpiStatus

}
