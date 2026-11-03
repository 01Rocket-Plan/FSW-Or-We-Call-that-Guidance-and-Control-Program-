@ OV3640 US 相機驅動（型號 OV3640、角色 US_CAM）
@ Core／Sim 介面一致；狀態機 1 Hz，擷取 100 Hz；影像只走 imageOut → sdLogger。
module Components {

  constant Ov3640_FRAME_BUF_MAX = 2048
  constant Ov3640_DECLARED_MODEL = CamModel.OV3640
  constant Ov3640_DECLARED_ROLE = CamRole.US_CAM

  port Ov3640ImageOut(
    index: U32
    length: U32
    data: [Ov3640_FRAME_BUF_MAX] U8
  )

  passive component Ov3640Driver {

    sync input port sched1Hz: Svc.Sched
    sync input port sched100Hz: Svc.Sched

    output port sccbWrite: Drv.I2c
    output port sccbWriteRead: Drv.I2cWriteRead
    output port imageOut: Ov3640ImageOut

    sync command POWER_ON()
    sync command RESET()
    sync command STOP()

    telemetry Status: CamStatus

    event StateChanged(from: CamState, to: CamState) \
      severity activity high \
      format "Ov3640Driver {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Ov3640Driver fault {} detail={}"

    param BUS_TIMEOUT_MISSES: U32 default 3
    param FRAME_TIMEOUT_TICKS: U32 default 20
    param ENTRIES_PER_TICK: U8 default 8
    param MODEL: CamModel default CamModel.OV3640
    param ROLE: CamRole default CamRole.US_CAM

    time get port timeCaller
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    event port logOut
    text event port logTextOut
    telemetry port tlmOut
    param get port prmGetOut
    param set port prmSetOut
  }

  passive component Ov3640DriverSim {

    sync input port sched1Hz: Svc.Sched
    sync input port sched100Hz: Svc.Sched
    output port imageOut: Ov3640ImageOut

    sync command POWER_ON()
    sync command RESET()
    sync command STOP()
    sync command SIM_INJECT(fault: U8)
    sync command SIM_SET_FRAME(enable: U8)

    telemetry Status: CamStatus

    event StateChanged(from: CamState, to: CamState) \
      severity activity high \
      format "Ov3640DriverSim {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Ov3640DriverSim fault {} detail={}"

    time get port timeCaller
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    event port logOut
    text event port logTextOut
    telemetry port tlmOut
  }

}
