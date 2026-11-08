@ 相機共用型別（CamState / CamFault / CamModel / CamRole / CamStatus / CamChunkFlag）
@ 與 Ov3640Core／Esp32CamCore 的 C++ 鏡射 enum 順序一致。
module Components {

  enum CamState {
    OFF = 0
    INIT = 1
    SELFTEST = 2
    READY = 3
    FAULT = 4
  }

  enum CamFault {
    NONE = 0
    ID_MISMATCH = 1
    TYPE_ROLE_MISMATCH = 2
    BUS_TIMEOUT = 3
    FRAME_TIMEOUT = 4
    @ 幀超過 FRAME_MAX_BYTES；事件 FrameTruncated 使用（CamStatus.fault 可不置位）
    FRAME_TRUNCATED = 5
  }

  enum CamModel {
    OV3640 = 0
    ESP32_CAM = 1
  }

  enum CamRole {
    US_CAM = 0
    DS_CAM = 1
  }

  @ imageOut 分塊旗標（可位元 OR：END|TRUNCATED）
  enum CamChunkFlag {
    NONE = 0
    END = 1
    TRUNCATED = 2
  }

  @ 遙測：camState、fault、frameCount、型號與角色（CamStatus 欄位不變）
  struct CamStatus {
    camState: CamState
    fault: CamFault
    frameCount: U32
    model: CamModel
    role: CamRole
  }

}
