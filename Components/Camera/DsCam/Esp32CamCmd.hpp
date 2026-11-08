// Esp32CamCmd：指令結果對應（host 可測；F´ 外殼轉成 Fw::CmdResponse）
#ifndef COMPONENTS_CAMERA_DSCAM_ESP32CAMCMD_HPP
#define COMPONENTS_CAMERA_DSCAM_ESP32CAMCMD_HPP

#include <cstdint>

namespace Components {

enum class Esp32CamCmdResult : uint8_t {
  OK = 0,
  EXECUTION_ERROR = 1
};

inline Esp32CamCmdResult esp32CamCmdResult(bool accepted) {
  return accepted ? Esp32CamCmdResult::OK : Esp32CamCmdResult::EXECUTION_ERROR;
}

}  // namespace Components
#endif
