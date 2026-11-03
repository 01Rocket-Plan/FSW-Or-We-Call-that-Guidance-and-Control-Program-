// SdCardSimDevice: in-memory SD media for Sim shell + host tests.
#ifndef RECORDER_SDCARDSIMDEVICE_HPP
#define RECORDER_SDCARDSIMDEVICE_HPP

#include "Components/Recorder/SdLogger/ISdMedia.hpp"
#include <cstdint>
#include <vector>

namespace Components {

enum class SdSimFaultKind : uint8_t {
  NONE = 0,
  FULL = 1,
  NACK = 2,
  WRITE_FAIL = 3,
  CARD_MISSING = 4,
};

class SdCardSimDevice : public ISdMedia {
 public:
  SdCardSimDevice();
  void setFault(SdSimFaultKind f) { m_fault = f; }
  SdSimFaultKind fault() const { return m_fault; }
  void setCapacity(size_t bytes) { m_capacity = bytes; }
  const std::vector<uint8_t>& image() const { return m_data; }
  void clear() { m_data.clear(); m_opened = false; m_status = SdMediaStatus::INIT; }

  bool open() override;
  bool append(const uint8_t* data, size_t len) override;
  bool sync() override;
  SdMediaStatus status() const override { return m_status; }
  size_t bytesWritten() const override { return m_data.size(); }

 private:
  std::vector<uint8_t> m_data;
  size_t m_capacity = 16u * 1024u * 1024u;  // 16 MiB sim default (UHS-I 16GB class scale-down)
  SdSimFaultKind m_fault = SdSimFaultKind::NONE;
  SdMediaStatus m_status = SdMediaStatus::INIT;
  bool m_opened = false;
};

}  // namespace Components
#endif
