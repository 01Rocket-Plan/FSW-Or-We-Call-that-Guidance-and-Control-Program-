// CamTypes：相機共用 C++ 型別。
// CamState／CamFault／CamModel／CamRole／CamChunkFlag／CamStatus 使用 FPP 自動碼，
// 此檔不再重複宣告，以免與 *EnumAc.hpp／CamStatusSerializableAc.hpp 衝突。
// Ov3640FrameView 不是 FPP 型別，仍在此定義。
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_CAMTYPES_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_CAMTYPES_HPP

#include "Components/Camera/Ov3640Driver/CamChunkFlagEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamFaultEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamModelEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamRoleEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamStateEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamStatusSerializableAc.hpp"
#include <cstdint>

namespace Components {

inline uint8_t camChunkFlagU8(CamChunkFlag f) {
  return static_cast<uint8_t>(static_cast<CamChunkFlag::T>(f));
}

struct Ov3640FrameView {
  const uint8_t* data = nullptr;
  uint32_t length = 0;
  bool complete = false;
};

}  // namespace Components

#endif
