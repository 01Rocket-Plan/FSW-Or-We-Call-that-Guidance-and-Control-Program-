@ DsCam / ESP32-CAM：DS 飛控側驅動（I2C → ICamLink）
@ 狀態機 1 Hz；擷取與 frameCount 100 Hz。ICamLink 無像素；影像走 imageOut 2KB 分塊（對齊 Ov3640ImageOut）。
@ 完整錄影亦可落 ESP32-CAM 本機 SD；地面預覽走獨立 IGroundPreviewLink（HTTP 144P）。
@ 型號 ESP32_CAM、角色 DS_CAM。共用 CamTypes／CamChunkFlag。
module Components {

  constant Esp32Cam_DECLARED_MODEL = CamModel.ESP32_CAM
  constant Esp32Cam_DECLARED_ROLE = CamRole.DS_CAM
  @ 單次 imageOut 分塊大小（bytes）；與 Ov3640_FRAME_BUF_MAX 對齊
  constant Esp32Cam_FRAME_BUF_MAX = 2048
  @ I2C 腳位／位址 placeholder：0xFF／0 表示未定，對齊 Anatoly 腳位表後再填
  constant Esp32Cam_I2C_PIN_PLACEHOLDER = 255
  constant Esp32Cam_I2C_ADDR_PLACEHOLDER = 0

  @ 影像分塊輸出（方案對齊 Ov3640ImageOut）。flags：CamChunkFlag（END=1, TRUNCATED=2）。
  port Esp32CamImageOut(
    frameNumber: U32
    chunkIndex: U32
    flags: U8
    ref data: Fw.Buffer
  )

  passive component Esp32Cam {

    @ 1 Hz 狀態機 tick
    sync input port sched1Hz: Svc.Sched
    @ 100 Hz 擷取／幀數 tick
    sync input port sched100Hz: Svc.Sched

    @ I2C 寫入（實體驅動；腳位／位址見參數）
    output port i2cWrite: Drv.I2c
    @ I2C 先寫後讀
    output port i2cWriteRead: Drv.I2cWriteRead
    @ 2KB 分塊影像（接本機 SD logger／下游）；不經遙測
    output port imageOut: Esp32CamImageOut

    sync command POWER_ON()
    sync command RESET()
    sync command CAPTURE_START()
    sync command CAPTURE_STOP()

    @ 電傳：僅 CamStatus（camState / fault / frameCount / model+role）
    telemetry Status: CamStatus

    event StateChanged(from: CamState, to: CamState) \
      severity activity high \
      format "Esp32Cam {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Esp32Cam fault {} detail={}"

    @ 幀超過 FRAME_MAX_BYTES：最後分塊標 TRUNCATED，本幀不計 frameCount，狀態維持 READY
    event FrameTruncated(frameNumber: U32, sentBytes: U32) \
      severity warning high \
      format "Esp32Cam frame {} truncated after {} bytes"

    param MODEL: CamModel default CamModel.ESP32_CAM
    param ROLE: CamRole default CamRole.DS_CAM
    @ 鏈路無回應門檻 N（SELFTEST）
    param MISS_LIMIT: U8 default 3
    @ 幀逾時門檻 M（100 Hz ticks）
    param FRAME_LIMIT: U32 default 20
    @ 超過此長度：最後分塊 TRUNCATED，不計 frameCount
    param FRAME_MAX_BYTES: U32 default 65536
    @ I2C 時脈 Hz（預設 400 kHz）
    param I2C_CLOCK_HZ: U32 default 400000
    @ I2C 7-bit 位址；0 = placeholder（等硬體定案）
    param I2C_ADDR: U8 default 0
    @ SDA GPIO；255 = placeholder（Anatoly 腳位表）
    param I2C_PIN_SDA: U8 default 255
    @ SCL GPIO；255 = placeholder（Anatoly 腳位表）
    param I2C_PIN_SCL: U8 default 255

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

  @ Sim：同一 Esp32CamCore + Esp32CamSimDevice（host／HITL）
  passive component Esp32CamSim {

    sync input port sched1Hz: Svc.Sched
    sync input port sched100Hz: Svc.Sched

    output port imageOut: Esp32CamImageOut

    sync command POWER_ON()
    sync command RESET()
    sync command CAPTURE_START()
    sync command CAPTURE_STOP()
    sync command SIM_INJECT(fault: U8)

    telemetry Status: CamStatus

    event StateChanged(from: CamState, to: CamState) \
      severity activity high \
      format "Esp32CamSim {} -> {}"

    event FaultRaised(code: CamFault, detail: U32) \
      severity warning high \
      format "Esp32CamSim fault {} detail={}"

    event FrameTruncated(frameNumber: U32, sentBytes: U32) \
      severity warning high \
      format "Esp32CamSim frame {} truncated after {} bytes"

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
