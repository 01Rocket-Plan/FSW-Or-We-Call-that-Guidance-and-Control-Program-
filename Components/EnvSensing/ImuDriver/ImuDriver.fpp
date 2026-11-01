module Components {

  @ DS IMU：ICM-20948 主 + ICM-20648 備援；1 kHz 非阻塞狀態機
  @ Base ID 0x4000（docs/conventions.md）；Sim 用 0x4080
  passive component ImuDriver {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    output port primaryWrite: EnvSensingBus.RegWrite
    output port primaryRead: EnvSensingBus.RegRead
    output port backupWrite: EnvSensingBus.RegWrite
    output port backupRead: EnvSensingBus.RegRead
    output port primarySpiWrite: EnvSensingBus.SpiRegWrite
    output port primarySpiRead: EnvSensingBus.SpiRegRead
    output port backupSpiWrite: EnvSensingBus.SpiRegWrite
    output port backupSpiRead: EnvSensingBus.SpiRegRead
    output port primaryI2c: Drv.I2c
    output port backupI2c: Drv.I2c
    output port primarySpi: Drv.SpiWriteRead
    output port backupSpi: Drv.SpiWriteRead

    output port imuOut: RocketTypes.ImuData

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry drvState: RocketTypes.DrvState
    telemetry activeSensor: U8
    telemetry busErrCount: U32
    telemetry stuckCount: U32
    telemetry failoverCount: U32

    event WhoAmIFail(
      sensor: U8
      got: U8
      expect: U8
    ) severity warning high format "IMU{} WHO_AM_I got 0x{x} expect 0x{x}"

    event BusError(
      sensor: U8
      count: U32
    ) severity warning high format "IMU{} bus errors={}"

    event StuckDetected(
      sensor: U8
      n: U32
    ) severity warning high format "IMU{} stuck for {} samples"

    event GyroSaturated(
      sensor: U8
    ) severity warning low format "IMU{} gyro saturated"

    event AccelSaturated(
      sensor: U8
    ) severity warning low format "IMU{} accel saturated"

    event Failover(
      fromSensor: U8
      toSensor: U8
    ) severity warning high format "IMU failover {} -> {}"

    event EnterFault(
      sensor: U8
      reason: U8
    ) severity warning high format "IMU{} enter FAULT reason={}"

    event EnterReady(
      sensor: U8
    ) severity activity high format "IMU{} READY"

    param stuckN: U32 default 50
    param busErrThreshold: U32 default 5
    param whoamiFailThreshold: U32 default 3
    param selftestSamples: U32 default 20
    param gyroSatThresh_rads: F32 default 34.0
    param accelSatThresh_mps2: F32 default 156.0
    param useSpi: bool default false
    param primaryI2cAddr: U8 default 0x69
    param backupI2cAddr: U8 default 0x68
    @ Body←sensor DCM row-major entries (identity default)
    param dcmP00: F32 default 1.0
    param dcmP01: F32 default 0.0
    param dcmP02: F32 default 0.0
    param dcmP10: F32 default 0.0
    param dcmP11: F32 default 1.0
    param dcmP12: F32 default 0.0
    param dcmP20: F32 default 0.0
    param dcmP21: F32 default 0.0
    param dcmP22: F32 default 1.0
    param dcmB00: F32 default 1.0
    param dcmB01: F32 default 0.0
    param dcmB02: F32 default 0.0
    param dcmB10: F32 default 0.0
    param dcmB11: F32 default 1.0
    param dcmB12: F32 default 0.0
    param dcmB20: F32 default 0.0
    param dcmB21: F32 default 0.0
    param dcmB22: F32 default 1.0
  }

  @ ImuDriver Sim + fault injection — Base ID 0x4080
  passive component ImuDriverSim {

    sync input port schedIn: Svc.Sched
    time get port timeCaller
    output port imuOut: RocketTypes.ImuData

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
    telemetry activeSensor: U8
    telemetry injectCode: U8

    event SimInject(
      code: U8
    ) severity activity low format "ImuDriverSim inject={}"

    event EnterReady(
      sensor: U8
    ) severity activity high format "ImuDriverSim IMU{} READY"

    event Failover(
      fromSensor: U8
      toSensor: U8
    ) severity warning high format "ImuDriverSim failover {} -> {}"

    event EnterFault(
      sensor: U8
      reason: U8
    ) severity warning high format "ImuDriverSim IMU{} FAULT reason={}"

    param stuckN: U32 default 50
    param busErrThreshold: U32 default 5
    param whoamiFailThreshold: U32 default 3
    param selftestSamples: U32 default 20
    param gyroSatThresh_rads: F32 default 34.0
    param accelSatThresh_mps2: F32 default 156.0
  }

}
