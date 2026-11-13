@ 相機共用型別（CamState / CamFault / CamModel / CamRole / CamStatus / CamChunkFlag）＋影像分塊埠
@ 這是唯一的定義：C++（Ov7670Core、Esp32CamCore、SdLogger）一律用 FPP 自動產生的型別，
@ 不准再手寫同名 C++ 型別（issue #36：手寫 CamTypes.hpp 和自動碼重複定義，真 F´ 編不過）。
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

  @ 上級相機 OV7670（SSS §7；Edward 2026-11-10 定案，取代 OV3640）
  enum CamModel {
    OV7670 = 0
    ESP32_CAM = 1
    @ 備援上級相機（只在 ov3640-alt 分支）
    OV3640 = 2
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

  @ 影像分塊輸出（相機 → sdLogger）。DS／US 兩顆相機共用同一個埠型別（issue #32：原本
  @ Esp32CamImageOut 在相機組和紀錄組各定義一次，整個 repo configure 失敗）。
  @ flags：CamChunkFlag 位元（END=1, TRUNCATED=2）。每塊最多 CAM_IMAGE_CHUNK_MAX bytes。
  port CamImageOut(
    frameNumber: U32
    chunkIndex: U32
    flags: U8
    ref data: Fw.Buffer
  )

  @ 單次 imageOut 分塊大小（bytes）
  constant CAM_IMAGE_CHUNK_MAX = 2048

  @ 遙測：camState、fault、frameCount、型號與角色（CamStatus 欄位不變）
  struct CamStatus {
    camState: CamState
    fault: CamFault
    frameCount: U32
    model: CamModel
    role: CamRole
  }

}
