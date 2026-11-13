module Components {

  @ DS IMU：ICM-20948（單顆）；1 kHz 非阻塞狀態機 OFF→INIT→SELFTEST→READY／FAULT
  @ Base ID 0x4000（docs/conventions.md）；Sim 用 0x4080
  @ 2026-11-10（Edward，#40／#42 D2）：ICM-20648 是「20948 太貴或缺貨時的替代料」，不是飛行中備援，
  @ 不進 main；替代版本在分支 EnvSensing/EdwardWang-Claude/icm20648-alt。
  @ 輸出 ImuSample 已是機體系（X 鼻／Y 右／Z=X×Y），由 dcm** 參數轉換；墊上靜置 ≈ (+9.81, 0, 0)。
  @ 事件只在狀態剛改變時記（飽和、FAULT／READY），遙測 1 Hz。
  passive component ImuDriver {

    sync input port schedIn: Svc.Sched
    time get port timeCaller

    output port busWrite: EnvSensingBus.RegWrite
    output port busRead: EnvSensingBus.RegRead

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
    telemetry busErrCount: U32
    telemetry stuckCount: U32
    telemetry faultReason: U8

    event GyroSaturated() severity warning low format "IMU gyro saturated"

    event AccelSaturated() severity warning low format "IMU accel saturated"

    event EnterFault(
      reason: U8
    ) severity warning high format "IMU enter FAULT reason={}"

    event EnterReady() severity activity high format "IMU READY"

    param stuckN: U32 default 50
    param busErrThreshold: U32 default 5
    param whoamiFailThreshold: U32 default 3
    param selftestSamples: U32 default 20
    param gyroSatThresh_rads: F32 default 34.0
    param accelSatThresh_mps2: F32 default 156.0
    param i2cAddr: U8 default 0x69
    @ 機體←晶片 DCM（row-major）。預設：ICM-20948 晶片 Z 沿鼻 → 機體 = (晶片 Z, 晶片 Y, −晶片 X)
    @ （跟 SSS plant/state.py 的 imu_to_body 同一張矩陣）。實際安裝方向不同時改這 9 個參數。
    param dcm00: F32 default 0.0
    param dcm01: F32 default 0.0
    param dcm02: F32 default 1.0
    param dcm10: F32 default 0.0
    param dcm11: F32 default 1.0
    param dcm12: F32 default 0.0
    param dcm20: F32 default -1.0
    param dcm21: F32 default 0.0
    param dcm22: F32 default 0.0
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
    telemetry injectCode: U8

    event SimInject(
      code: U8
    ) severity activity low format "ImuDriverSim inject={}"

    event EnterReady() severity activity high format "ImuDriverSim READY"

    event EnterFault(
      reason: U8
    ) severity warning high format "ImuDriverSim FAULT reason={}"

    param stuckN: U32 default 50
    param busErrThreshold: U32 default 5
    param whoamiFailThreshold: U32 default 3
    param selftestSamples: U32 default 20
    param gyroSatThresh_rads: F32 default 34.0
    param accelSatThresh_mps2: F32 default 156.0
  }

}
