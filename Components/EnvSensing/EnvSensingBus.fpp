module EnvSensingBus {

  enum BusStatus {
    OK
    NACK
    ERROR
  }

  @ Abstract register write — deployment adapts to Drv.I2c / Drv.Spi
  port RegWrite(
    devAddr: U8
    regAddr: U8
    ref writeBuffer: Fw.Buffer
  ) -> BusStatus

  @ Abstract register read
  port RegRead(
    devAddr: U8
    regAddr: U8
    ref readBuffer: Fw.Buffer
  ) -> BusStatus

  port SpiRegWrite(
    regAddr: U8
    ref writeBuffer: Fw.Buffer
  ) -> BusStatus

  port SpiRegRead(
    regAddr: U8
    ref readBuffer: Fw.Buffer
  ) -> BusStatus

  @ Sim fault injection code (0..5; 6 = bank1 ST fail)
  port FaultInject(
    code: U8
  )

}
