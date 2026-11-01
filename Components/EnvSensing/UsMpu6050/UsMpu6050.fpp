module Components {

  @ US MPU6050：僅記錄、原生軸；Base ID 0x4200；Sim 0x4280
  passive component UsMpu6050 {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    output port busWrite: EnvSensingBus.RegWrite
    output port busRead: EnvSensingBus.RegRead
    output port i2c: Drv.I2c

    output port mpuOut: RocketTypes.Mpu6050Data

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry drvState: RocketTypes.DrvState
    telemetry busErrCount: U32
    telemetry stuckCount: U32

    event WhoAmIFail(
      got: U8
      expect: U8
    ) severity warning high format "MPU6050 WHO_AM_I got 0x{x} expect 0x{x}"

    event BusError(
      count: U32
    ) severity warning high format "MPU6050 bus errors={}"

    event StuckDetected(
      n: U32
    ) severity warning high format "MPU6050 stuck for {} samples"

    event GyroSaturated severity warning low format "MPU6050 gyro saturated"
    event AccelSaturated severity warning low format "MPU6050 accel saturated"

    event EnterFault(
      reason: U8
    ) severity warning high format "MPU6050 FAULT reason={}"

    event EnterReady severity activity high format "MPU6050 READY"

    param stuckN: U32 default 50
    param busErrThreshold: U32 default 5
    param whoamiFailThreshold: U32 default 3
    param selftestSamples: U32 default 10
    param gyroSatThresh_rads: F32 default 34.0
    param accelSatThresh_mps2: F32 default 156.0
    param i2cAddr: U8 default 0x68
  }

  @ UsMpu6050 Sim — Base ID 0x4280
  passive component UsMpu6050Sim {

    sync input port schedIn: Svc.Sched
    time get port timeCaller
    output port mpuOut: RocketTypes.Mpu6050Data
    sync input port injectFault: EnvSensingBus.FaultInject

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry drvState: RocketTypes.DrvState
    telemetry injectCode: U8

    event SimInject(
      code: U8
    ) severity activity low format "UsMpu6050Sim inject={}"

    event EnterReady severity activity high format "UsMpu6050Sim READY"

    event EnterFault(
      reason: U8
    ) severity warning high format "UsMpu6050Sim FAULT reason={}"

    param stuckN: U32 default 50
    param busErrThreshold: U32 default 5
    param whoamiFailThreshold: U32 default 3
    param selftestSamples: U32 default 10
    param gyroSatThresh_rads: F32 default 34.0
    param accelSatThresh_mps2: F32 default 156.0
  }

}
