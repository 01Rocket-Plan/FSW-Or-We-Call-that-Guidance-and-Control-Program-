// ILocalSdStore：ESP32-CAM 板上 SD 卡完整錄影抽象
// 規則：錄影留在相機模組本機 SD，不進飛控遙測、不經 ICamLink。
#ifndef COMPONENTS_CAMERA_DSCAM_ILOCALSDSTORE_HPP
#define COMPONENTS_CAMERA_DSCAM_ILOCALSDSTORE_HPP

#include "Components/Camera/DsCam/Esp32CamFrame.hpp"
#include <cstdint>

namespace Components {

class ILocalSdStore {
 public:
  virtual ~ILocalSdStore() = default;
  // 寫入一幀完整錄影資料；回傳 false 表示丟棄／未寫入。
  virtual bool writeFrame(const CamFrameHeader& hdr, const uint8_t* data) = 0;
};

// Host／Sim 用記憶體計數 stub（不真的存檔）
class LocalSdStoreStub : public ILocalSdStore {
 public:
  bool writeFrame(const CamFrameHeader& hdr, const uint8_t* data) override {
    (void)data;
    m_last = hdr;
    m_frames++;
    m_bytes += hdr.bytes;
    return true;
  }
  uint32_t frames() const { return m_frames; }
  uint32_t bytes() const { return m_bytes; }
  CamFrameHeader last() const { return m_last; }

 private:
  uint32_t m_frames = 0;
  uint32_t m_bytes = 0;
  CamFrameHeader m_last{};
};

}  // namespace Components
#endif
