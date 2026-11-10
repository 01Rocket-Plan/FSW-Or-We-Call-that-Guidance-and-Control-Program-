// CamFrameView：幀來源回傳的「借用視圖」（指標＋長度），C++ 專用。
// 這不是 FPP 型別的鏡射（FPP 不能表達指標），所以放在這裡；
// CamState／CamFault／CamModel／CamRole／CamStatus／CamChunkFlag 一律用 FPP 自動碼。
#ifndef COMPONENTS_CAMERA_CAMTYPES_CAMFRAMEVIEW_HPP
#define COMPONENTS_CAMERA_CAMTYPES_CAMFRAMEVIEW_HPP

#include <cstdint>

namespace Components {

struct CamFrameView {
  const uint8_t* data = nullptr;
  uint32_t length = 0;
  bool complete = false;
};

}  // namespace Components

#endif
