// CamTypes：相機共用 C++ 型別（與 CamTypes.fpp 順序一致）
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_CAMTYPES_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_CAMTYPES_HPP

#include <cstdint>

namespace Components {

enum class CamState : uint8_t { OFF = 0, INIT = 1, SELFTEST = 2, READY = 3, FAULT = 4 };

enum class CamFault : uint8_t {
  NONE = 0,
  ID_MISMATCH = 1,
  TYPE_ROLE_MISMATCH = 2,
  BUS_TIMEOUT = 3,
  FRAME_TIMEOUT = 4,
  FRAME_TRUNCATED = 5
};

enum class CamModel : uint8_t { OV3640 = 0, ESP32_CAM = 1 };

enum class CamRole : uint8_t { US_CAM = 0, DS_CAM = 1 };

// Bit flags for imageOut chunks (END | TRUNCATED allowed)
enum class CamChunkFlag : uint8_t { NONE = 0, END = 1, TRUNCATED = 2 };

inline uint8_t camChunkFlagU8(CamChunkFlag f) { return static_cast<uint8_t>(f); }

struct CamStatus {
  CamState camState = CamState::OFF;
  CamFault fault = CamFault::NONE;
  uint32_t frameCount = 0;
  CamModel model = CamModel::OV3640;
  CamRole role = CamRole::US_CAM;
};

struct Ov3640FrameView {
  const uint8_t* data = nullptr;
  uint32_t length = 0;
  bool complete = false;
};

}  // namespace Components

#endif
