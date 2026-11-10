// Esp32CamImageStreamer：2KB Fw.Buffer 風格分塊（對齊 CamImageOut 方案）
// 共用 CamChunkFlag；驅動內只保留一個 chunk scratch，不持有整張 JPEG。
#ifndef COMPONENTS_CAMERA_DSCAM_ESP32CAMIMAGESTREAMER_HPP
#define COMPONENTS_CAMERA_DSCAM_ESP32CAMIMAGESTREAMER_HPP

// 型別來自 FPP 自動碼（../CamTypes/CamTypes.fpp，issue #36）
#include "Components/Camera/CamTypes/CamChunkFlagEnumAc.hpp"
#include "Components/Camera/CamTypes/CamFaultEnumAc.hpp"
#include "Components/Camera/CamTypes/CamModelEnumAc.hpp"
#include "Components/Camera/CamTypes/CamRoleEnumAc.hpp"
#include "Components/Camera/CamTypes/CamStateEnumAc.hpp"
#include "Components/Camera/CamTypes/CamStatusSerializableAc.hpp"
#include <cstdint>
#include <cstring>

namespace Components {

class ICamImageChunkSink {
 public:
  virtual ~ICamImageChunkSink() = default;
  virtual void onImageChunk(uint32_t frameNumber, uint32_t chunkIndex, uint8_t flags,
                            const uint8_t* data, uint32_t length) = 0;
};

class Esp32CamImageStreamer {
 public:
  static constexpr uint32_t kChunkBytes = 2048;
  static constexpr uint32_t kDefaultFrameMaxBytes = 65536;

  void setSink(ICamImageChunkSink* sink) { m_sink = sink; }
  void setFrameMaxBytes(uint32_t n) { m_frameMaxBytes = (n == 0) ? kDefaultFrameMaxBytes : n; }
  uint32_t frameMaxBytes() const { return m_frameMaxBytes; }

  // true = 完整接受（呼叫端應 ++frameCount）；false = 截斷／拒絕（不計 frameCount）
  bool streamFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) {
    if (m_sink == nullptr || data == nullptr || length == 0) {
      return false;
    }
    const uint32_t limit = (length < m_frameMaxBytes) ? length : m_frameMaxBytes;
    const bool truncated = length > m_frameMaxBytes;
    uint32_t sent = 0;
    uint32_t chunkIdx = 0;
    while (sent < limit) {
      uint32_t n = limit - sent;
      if (n > kChunkBytes) {
        n = kChunkBytes;
      }
      std::memcpy(m_scratch, data + sent, n);
      sent += n;
      uint8_t flags = 0;
      if (sent >= limit) {
        flags = static_cast<uint8_t>(CamChunkFlag::END);
        if (truncated) {
          flags = static_cast<uint8_t>(static_cast<uint8_t>(CamChunkFlag::END) |
                                       static_cast<uint8_t>(CamChunkFlag::TRUNCATED));
        }
      }
      m_sink->onImageChunk(frameNumber, chunkIdx, flags, m_scratch, n);
      ++chunkIdx;
    }
    m_lastChunkCount = chunkIdx;
    m_lastTruncated = truncated;
    return !truncated;
  }

  uint32_t lastChunkCount() const { return m_lastChunkCount; }
  bool lastTruncated() const { return m_lastTruncated; }

 private:
  ICamImageChunkSink* m_sink = nullptr;
  uint32_t m_frameMaxBytes = kDefaultFrameMaxBytes;
  uint32_t m_lastChunkCount = 0;
  bool m_lastTruncated = false;
  uint8_t m_scratch[kChunkBytes]{};
};

}  // namespace Components
#endif
