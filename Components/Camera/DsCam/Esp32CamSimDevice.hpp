// Esp32CamSimDevice：ESP32-CAM ICamLink 模擬（給 host 測試）
// 以位元組隊列回應握手與幀通知；可注入故障。影像像素不經此鏈路。
// 完整錄影 → ILocalSdStore；地面預覽 → IGroundPreviewLink。
#ifndef COMPONENTS_CAMERA_DSCAM_ESP32CAMSIMDEVICE_HPP
#define COMPONENTS_CAMERA_DSCAM_ESP32CAMSIMDEVICE_HPP

#include "Components/Camera/Ov3640Driver/CamTypes.hpp"
#include "Components/Camera/DsCam/Esp32CamFrame.hpp"
#include "Components/Camera/DsCam/ICamLink.hpp"
#include "Components/Camera/DsCam/IGroundPreviewLink.hpp"
#include "Components/Camera/DsCam/ILocalSdStore.hpp"
#include <cstdint>

namespace Components {

enum class Esp32CamSimFault : uint8_t {
  NONE = 0,
  SILENT = 1,
  NO_FRAMES = 2,
  WRONG_MODEL = 3,
  WRONG_ROLE = 4
};

class Esp32CamSimDevice : public ICamLink {
 public:
  Esp32CamSimDevice();

  void configure(CamModel model, CamRole role);
  void setFault(Esp32CamSimFault f) { m_fault = f; }
  void setLocalSd(ILocalSdStore* sd) { m_sd = sd; }
  void setPreviewLink(IGroundPreviewLink* preview) { m_preview = preview; }

  void advance();

  uint32_t send(const uint8_t* buf, uint32_t len) override;
  uint32_t recv(uint8_t* buf, uint32_t cap) override;

  uint32_t sentBytes() const { return m_sentBytes; }
  uint32_t framesPushed() const { return m_framesPushed; }
  bool capturing() const { return m_capturing; }
  // ICamLink 上送出的最大單次 payload（應只有 header，無像素 bulk）
  uint32_t maxNotifyPayload() const { return m_maxNotifyPayload; }

 private:
  void push(const uint8_t* buf, uint32_t len);
  void pushInfo();
  void pushFrame(bool selftest);

  CamModel m_model = CamModel::ESP32_CAM;
  CamRole m_role = CamRole::DS_CAM;
  Esp32CamSimFault m_fault = Esp32CamSimFault::NONE;
  ILocalSdStore* m_sd = nullptr;
  IGroundPreviewLink* m_preview = nullptr;

  uint8_t m_rx[256];
  uint32_t m_rxHead = 0;
  uint32_t m_rxTail = 0;

  uint32_t m_sentBytes = 0;
  uint32_t m_framesPushed = 0;
  uint32_t m_tickCounter = 0;
  uint32_t m_maxNotifyPayload = 0;
  bool m_capturing = false;
};

}  // namespace Components
#endif
