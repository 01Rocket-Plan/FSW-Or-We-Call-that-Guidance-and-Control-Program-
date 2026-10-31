# Host fpp-check stubs until lib/fprime is vendored (MCU group).
# Signatures aligned with F´ v4.4.1 Drv/Fw ports where applicable.

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
    I2C_OPEN_ERR
    I2C_OTHER_ERR
  }

  enum SpiStatus {
    SPI_OK
    SPI_OPEN_ERR
    SPI_CONFIG_ERR
    SPI_MISMATCH_ERR
    SPI_WRITE_ERR
    SPI_OTHER_ERR
  }

  port I2c(
    addr: U32
    ref serBuffer: Fw.Buffer
  ) -> I2cStatus

  port I2cWriteRead(
    addr: U32
    ref writeBuffer: Fw.Buffer
    ref readBuffer: Fw.Buffer
  ) -> I2cStatus

  port SpiWriteRead(
    ref writeBuffer: Fw.Buffer
    ref readBuffer: Fw.Buffer
  ) -> SpiStatus

}
