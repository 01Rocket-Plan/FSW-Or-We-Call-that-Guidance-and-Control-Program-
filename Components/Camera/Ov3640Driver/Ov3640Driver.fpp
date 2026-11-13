@ OV3640 US 相機驅動（型號 OV3640、角色 US_CAM）
@ 備援方案（分支 camera/EdwardWang-Claude/ov3640-alt，不合 main；主案是 OV7670）。
@ 型別與影像埠來自 Components/Camera/CamTypes（FPP 自動碼）。
@ Core／Sim 介面一致；狀態機 1 Hz，擷取 100 Hz；影像只走 imageOut → sdLogger（2KB 分塊串流）。
module Components {

  @ 單次 imageOut 分塊大小（bytes）。US_CAM 以 Fw.Buffer 串流給 sdLogger，驅動內不保留整張 JPEG。
  constant Ov3640_FRAME_BUF_MAX = 2048
  constant Ov3640_DECLARED_MODEL = CamModel.OV3640
  constant Ov3640_DECLARED_ROLE = CamRole.US_CAM


  @ US OV3640 驅動（SCCB 設定 + DVP 擷取）
  passive component Ov3640Driver {

    @ 1 Hz：握手／健康／FAULT 自動重試（不接 1 kHz）
    sync input port sched1Hz: Svc.Sched

    @ 100 Hz：初始化批次、自檢、幀輪詢（不接 1 kHz）
    sync input port sched100Hz: Svc.Sched

    output port sccbWrite: Drv.I2c
    output port sccbWriteRead: Drv.I2cWriteRead
    output port imageOut: CamImageOut

    sync command POWER_ON()
    sync command RESET()
    sync command STOP()
    sync command CAPTURE_START()
    sync command CAPTURE_STOP()

    telemetry Status: CamStatus

    event StateChanged(from: CamState, to: CamState) \
      severity activity high \
      format "Ov3640Driver {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Ov3640Driver fault {} detail={}"

    @ FAULT 後每次自動重進 INIT（attempt=1..R）
    event RecoverAttempt(attempt: U32, limit: U32) \
      severity activity high \
      format "Ov3640Driver recover attempt {}/{}"

    @ 超過 FRAME_MAX_BYTES：最後分塊標 TRUNCATED，本幀不計入 frameCount
    event FrameTruncated(frameNumber: U32, bytesSent: U32) \
      severity warning high \
      format "Ov3640Driver FRAME_TRUNCATED frame={} bytesSent={}"

    param BUS_TIMEOUT_MISSES: U32 default 3
    param FRAME_TIMEOUT_TICKS: U32 default 20
    param ENTRIES_PER_TICK: U8 default 8
    param MODEL: CamModel default CamModel.OV3640
    param ROLE: CamRole default CamRole.US_CAM
    param SCCB_ADDR: U32 default 0x3C
    @ FAULT 自動重試：1 Hz 每 K tick 重跑 INIT
    param FAULT_RECOVER_K: U32 default 5
    @ FAULT 自動重試上限；超過後鎖住，僅 RESET 可清
    param FAULT_RECOVER_R: U32 default 3
    @ 單幀最多串流 bytes；超過則截斷並 FrameTruncated
    param FRAME_MAX_BYTES: U32 default 65536

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
    output port imageOut: CamImageOut

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

    event RecoverAttempt(attempt: U32, limit: U32) \
      severity activity high \
      format "Ov3640DriverSim recover attempt {}/{}"

    event FrameTruncated(frameNumber: U32, bytesSent: U32) \
      severity warning high \
      format "Ov3640DriverSim FRAME_TRUNCATED frame={} bytesSent={}"

    param BUS_TIMEOUT_MISSES: U32 default 3
    param FRAME_TIMEOUT_TICKS: U32 default 20
    param ENTRIES_PER_TICK: U8 default 8
    param MODEL: CamModel default CamModel.OV3640
    param ROLE: CamRole default CamRole.US_CAM
    param FAULT_RECOVER_K: U32 default 5
    param FAULT_RECOVER_R: U32 default 3
    param FRAME_MAX_BYTES: U32 default 65536

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
