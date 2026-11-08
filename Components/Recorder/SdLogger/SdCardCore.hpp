// SdCardCore: framed append API over ISdMedia (BME680-style pure core).
#ifndef RECORDER_SDCARDCORE_HPP
#define RECORDER_SDCARDCORE_HPP

#include "Components/Recorder/SdLogger/ISdMedia.hpp"
#include <cstddef>
#include <cstdint>

namespace Components {

struct SdRecordHeader {
  uint16_t channelId = 0;
  uint32_t loggerSeq = 0;
  uint32_t producerSeq = 0;
  double t_s = 0.0;
};

class SdCardCore {
 public:
  explicit SdCardCore(ISdMedia& media);
  bool init();
  //! Frame + append one record. Returns false on media error / full.
  bool writeRecord(const SdRecordHeader& hdr, const uint8_t* payload, uint16_t payloadLen);
  bool sync();
  SdMediaStatus status() const { return m_media.status(); }
  uint32_t errorCount() const { return m_errors; }
  uint32_t writeCount() const { return m_writes; }
  uint32_t writePoolInUse() const { return m_poolInUse; }
  uint32_t writePoolHwm() const { return m_poolHwm; }
  void setPoolSize(uint32_t n) { m_poolSize = n ? n : 1; }
  uint32_t poolSize() const { return m_poolSize; }

  static constexpr uint32_t kMagic = 0x524C4F47u;  // 'RLOG'
  static constexpr uint16_t kMaxPayload = 2056;  // 2048 image + 8B meta

 private:
  ISdMedia& m_media;
  uint32_t m_errors = 0;
  uint32_t m_writes = 0;
  uint32_t m_poolSize = 8;
  uint32_t m_poolInUse = 0;
  uint32_t m_poolHwm = 0;
};

}  // namespace Components
#endif
