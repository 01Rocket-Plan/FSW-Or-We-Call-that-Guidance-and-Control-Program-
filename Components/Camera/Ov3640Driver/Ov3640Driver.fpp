@ OV3640 US 相機驅動（型號 OV3640、角色 US_CAM）
@ Core／Sim 介面一致；狀態機 1 Hz，擷取 100 Hz；影像只走 imageOut → sdLogger。
module Components {

  constant Ov3640_FRAME_BUF_MAX = 2048
  constant Ov3640_DECLARED_MODEL = CamModel.OV3640
  constant Ov3640_DECLARED_ROLE = CamRole.US_CAM

  @ 影像輸出（接 sdLogger）。使用 Fw.Buffer，禁止匿名陣列 port 參數。
  port Ov3640ImageOut(
    index: U32
    ref data: Fw.Buffer
  )

  @ US OV3640 驅動（SCCB 設定 + DVP 擷取）
  passive component Ov3640Driver {

    @ 1 Hz：握手／健康（不接 1 kHz）
    sync input port sched1Hz: Svc.Sched

    @ 100 Hz：初始化批次、自檢、幀輪詢（不接 1 kHz）
    sync input port sched100Hz: Svc.Sched

    @ SCCB 寫
    output port sccbWrite: Drv.I2c

    @ SCCB 讀（write-then-read）
    output port sccbWriteRead: Drv.I2cWriteRead

    @ 影像 → sdLogger（唯一影像出口）
    output port imageOut: Ov3640ImageOut

    sync command POWER_ON()
    sync command RESET()
    sync command STOP()
    @ 僅 READY 可開始擷取；否則 EXECUTION_ERROR
    sync command CAPTURE_START()
    @ 停止擷取（保持 READY）
    sync command CAPTURE_STOP()

    @ 遙測僅 CamStatus（camState / fault / frameCount / model / role）
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
    @ SCCB 7-bit 位址（OV3640 常見 0x3C）
    param SCCB_ADDR: U32 default 0x3C

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

  @ Ov3640Driver 的 Sim 版：同一 Core，接 Ov3640SimDevice
  passive component Ov3640DriverSim {

    sync input port sched1Hz: Svc.Sched
    sync input port sched100Hz: Svc.Sched
    output port imageOut: Ov3640ImageOut

    sync command POWER_ON()
    sync command RESET()
    sync command STOP()
    sync command CAPTURE_START()
    sync command CAPTURE_STOP()
    sync command SIM_INJECT(fault: U8)
    sync command SIM_SET_FRAME(enable: U8)

    telemetry Status: CamStatus

    event StateChanged(from: CamState, to: CamState) \
      severity activity high \
      format "Ov3640DriverSim {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Ov3640DriverSim fault {} detail={}"

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

}
