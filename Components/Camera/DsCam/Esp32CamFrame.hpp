// Esp32CamFrame：DS ESP32-CAM 幀元資料（不含像素）
// 共用 CamState／CamFault／CamStatus 請用 Ov3640Driver/CamTypes.hpp，勿在此重複定義。
#ifndef COMPONENTS_CAMERA_DSCAM_ESP32CAMFRAME_HPP
#define COMPONENTS_CAMERA_DSCAM_ESP32CAMFRAME_HPP

#include <cstdint>

namespace Components {

struct CamFrameHeader {
  uint32_t index = 0;
  uint32_t bytes = 0;
  uint16_t width = 0;
  uint16_t height = 0;
};

}  // namespace Components
#endif
