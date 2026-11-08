#include "Components/Recorder/SdLogger/SdCardSimDevice.hpp"

namespace Components {

SdCardSimDevice::SdCardSimDevice() = default;

bool SdCardSimDevice::open() {
  if (m_fault == SdSimFaultKind::CARD_MISSING) {
    m_status = SdMediaStatus::CARD_MISSING;
    m_opened = false;
    return false;
  }
  m_opened = true;
  m_status = SdMediaStatus::OK;
  return true;
}

bool SdCardSimDevice::append(const uint8_t* data, size_t len) {
  if (!m_opened) {
    m_status = SdMediaStatus::CARD_MISSING;
    return false;
  }
  if (m_fault == SdSimFaultKind::CARD_MISSING) {
    m_status = SdMediaStatus::CARD_MISSING;
    return false;
  }
  if (m_fault == SdSimFaultKind::NACK || m_fault == SdSimFaultKind::WRITE_FAIL) {
    m_status = SdMediaStatus::IO_ERROR;
    return false;
  }
  if (m_fault == SdSimFaultKind::FULL || m_data.size() + len > m_capacity) {
    m_status = SdMediaStatus::FULL;
    return false;
  }
  m_status = SdMediaStatus::BUSY;
  m_data.insert(m_data.end(), data, data + len);
  m_status = SdMediaStatus::OK;
  return true;
}

bool SdCardSimDevice::sync() {
  if (!m_opened) return false;
  if (m_fault == SdSimFaultKind::WRITE_FAIL || m_fault == SdSimFaultKind::NACK) {
    m_status = SdMediaStatus::IO_ERROR;
    return false;
  }
  return true;
}

}  // namespace Components
