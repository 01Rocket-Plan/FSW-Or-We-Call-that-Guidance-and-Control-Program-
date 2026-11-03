@ 相機共用型別（CamState / CamFault / CamModel / CamRole / CamStatus）
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
  }

  enum CamModel {
    OV3640 = 0
    ESP32_CAM = 1
  }

  enum CamRole {
    US_CAM = 0
    DS_CAM = 1
  }

  @ 遙測：state、fault、frameCount、型號與角色（分兩個列舉欄位）
  struct CamStatus {
    state: CamState
    fault: CamFault
    frameCount: U32
    model: CamModel
    role: CamRole
  }

}
