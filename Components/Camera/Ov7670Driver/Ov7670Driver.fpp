@ OV7670 US 相機驅動（型號 OV7670、角色 US_CAM；SSS §7，Edward 2026-11-10 定案）
@ Core／Sim 介面一致；狀態機 1 Hz，擷取 100 Hz；影像只走 imageOut → sdLogger（2KB 分塊串流）。
@ OV7670 沒有 JPEG：感測器出 QCIF 176×144 YUV422，驅動只取 Y → 灰階一幀 25,344 B（13 塊）；1 fps（Edward 2026-11-14）。
@ 硬體：SCCB 7-bit 位址 0x21；XCLK 10–48 MHz（典型 24 MHz）；DVP 8-bit＋VSYNC／HREF／PCLK。
module Components {

  constant Ov7670_DECLARED_MODEL = CamModel.OV7670
  constant Ov7670_DECLARED_ROLE = CamRole.US_CAM
  @ QCIF 176×144 灰階一整幀（bytes，只有 Y）
  constant Ov7670_FRAME_BYTES = 25344

  @ US OV7670 驅動（SCCB 設定 + DVP 擷取）
  passive component Ov7670Driver {

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
      format "Ov7670Driver {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Ov7670Driver fault {} detail={}"

    @ FAULT 後每次自動重進 INIT（attempt=1..R）
    event RecoverAttempt(attempt: U32, limit: U32) \
      severity activity high \
      format "Ov7670Driver recover attempt {}/{}"

    @ 超過 FRAME_MAX_BYTES：最後分塊標 TRUNCATED，本幀不計入 frameCount
    event FrameTruncated(frameNumber: U32, bytesSent: U32) \
      severity warning high \
      format "Ov7670Driver FRAME_TRUNCATED frame={} bytesSent={}"

    param BUS_TIMEOUT_MISSES: U32 default 3
    param FRAME_TIMEOUT_TICKS: U32 default 20
    param ENTRIES_PER_TICK: U8 default 8
    param MODEL: CamModel default CamModel.OV7670
    param ROLE: CamRole default CamRole.US_CAM
    @ SCCB 7-bit 位址（OV7670 規格書：寫 0x42／讀 0x43 → 0x21）
    param SCCB_ADDR: U32 default 0x21
    @ FAULT 自動重試：1 Hz 每 K tick 重跑 INIT
    param FAULT_RECOVER_K: U32 default 5
    @ FAULT 自動重試上限；超過後鎖住，僅 RESET 可清
    param FAULT_RECOVER_R: U32 default 3
    @ 單幀最多串流 bytes（灰階）；超過則截斷並 FrameTruncated（預設＝一整張 QCIF 灰階）
    param FRAME_MAX_BYTES: U32 default Ov7670_FRAME_BYTES
    @ 幀率上限：兩次轉存之間至少隔幾個 100 Hz 拍（100＝1 fps；0＝不限）
    param FRAME_PERIOD_TICKS: U32 default 100

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

  passive component Ov7670DriverSim {

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
      format "Ov7670DriverSim {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Ov7670DriverSim fault {} detail={}"

    event RecoverAttempt(attempt: U32, limit: U32) \
      severity activity high \
      format "Ov7670DriverSim recover attempt {}/{}"

    event FrameTruncated(frameNumber: U32, bytesSent: U32) \
      severity warning high \
      format "Ov7670DriverSim FRAME_TRUNCATED frame={} bytesSent={}"

    param BUS_TIMEOUT_MISSES: U32 default 3
    param FRAME_TIMEOUT_TICKS: U32 default 20
    param ENTRIES_PER_TICK: U8 default 8
    param MODEL: CamModel default CamModel.OV7670
    param ROLE: CamRole default CamRole.US_CAM
    param FAULT_RECOVER_K: U32 default 5
    param FAULT_RECOVER_R: U32 default 3
    param FRAME_MAX_BYTES: U32 default Ov7670_FRAME_BYTES
    param FRAME_PERIOD_TICKS: U32 default 100

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
