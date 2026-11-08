// IGroundPreviewLink：飛行中地面預覽抽象（144P、1～3 FPS、HTTP 成幀）
// 規則：與 ICamLink 完全分離；禁止經 ICamLink 或飛控遙測傳預覽影像。
// 實體層（WiFi / RF）留空，由無線組定案後填入。
#ifndef COMPONENTS_CAMERA_DSCAM_IGROUNDPREVIEWLINK_HPP
#define COMPONENTS_CAMERA_DSCAM_IGROUNDPREVIEWLINK_HPP

#include "Components/Camera/DsCam/Esp32CamFrame.hpp"
#include <cstdint>

namespace Components {

struct GroundPreviewSpec {
  static constexpr uint16_t kWidth = 256;   // 144P 寬（近似 256x144）
  static constexpr uint16_t kHeight = 144;
  static constexpr uint8_t kFpsMin = 1;
  static constexpr uint8_t kFpsMax = 3;
};

class IGroundPreviewLink {
 public:
  virtual ~IGroundPreviewLink() = default;
  // 非阻塞送出一幀預覽（HTTP 成幀由實作處理）；回傳 false 表示未送出。
  virtual bool sendPreview(const CamFrameHeader& hdr, const uint8_t* data) = 0;
};

// 實體層 stub：接受呼叫但不送出，供 host 測試隔離驗證
class GroundPreviewLinkStub : public IGroundPreviewLink {
 public:
  bool sendPreview(const CamFrameHeader& hdr, const uint8_t* data) override {
    (void)data;
    m_last = hdr;
    m_count++;
    return true;
  }
  uint32_t count() const { return m_count; }
  CamFrameHeader last() const { return m_last; }

 private:
  uint32_t m_count = 0;
  CamFrameHeader m_last{};
};

}  // namespace Components
#endif
