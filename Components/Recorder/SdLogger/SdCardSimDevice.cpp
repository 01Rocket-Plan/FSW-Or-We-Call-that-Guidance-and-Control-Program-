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
  if (m_busyWindows > 0u) {
    // 注入的卡片忙碌（寫入逾時情境）
    ++m_busyRejects;
    m_status = SdMediaStatus::BUSY;
    return false;
  }
  if (m_writeBps != 0u) {
    if (m_credit <= 0) {
      // 這個視窗的寫入額度用完：留到下一個視窗
      ++m_busyRejects;
      m_status = SdMediaStatus::BUSY;
      return false;
    }
    m_credit -= static_cast<int64_t>(len);  // 可透支一筆，下個視窗扣回
  }
  m_status = SdMediaStatus::BUSY;
  m_data.insert(m_data.end(), data, data + len);
  m_status = SdMediaStatus::OK;
  return true;
}

void SdCardSimDevice::advance() {
  if (m_busyWindows > 0u) --m_busyWindows;
  const int64_t budget = windowBudget();
  m_credit += budget;
  if (m_credit > budget) m_credit = budget;  // 閒置不累積：一個視窗最多寫一個視窗的量
  if (m_status == SdMediaStatus::BUSY && m_busyWindows == 0u) m_status = SdMediaStatus::OK;
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
