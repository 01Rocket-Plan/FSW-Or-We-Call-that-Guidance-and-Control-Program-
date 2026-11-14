#include "Components/Camera/DsCam/Esp32CamSimDevice.hpp"

namespace Components {

namespace {
constexpr uint8_t kCmdPing = 0xA1;
constexpr uint8_t kCmdCapStart = 0xB1;
constexpr uint8_t kCmdCapStop = 0xB2;
constexpr uint8_t kRspPong = 0xA2;
constexpr uint8_t kRspFrame = 0xC0;
constexpr uint8_t kRspInfo = 0xD0;
}  // namespace

Esp32CamSimDevice::Esp32CamSimDevice() = default;

void Esp32CamSimDevice::configure(CamModel model, CamRole role) {
  m_model = model;
  m_role = role;
}

void Esp32CamSimDevice::push(const uint8_t* buf, uint32_t len) {
  if (len > m_maxNotifyPayload) {
    m_maxNotifyPayload = len;
  }
  for (uint32_t i = 0; i < len; ++i) {
    uint32_t next = (m_rxTail + 1) % sizeof(m_rx);
    if (next == m_rxHead) {
      return;
    }
    m_rx[m_rxTail] = buf[i];
    m_rxTail = next;
  }
}

void Esp32CamSimDevice::pushInfo() {
  CamModel repModel = m_model;
  if (m_fault == Esp32CamSimFault::WRONG_MODEL) {
    repModel = CamModel::OV7670;
  }
  if (m_fault == Esp32CamSimFault::WRONG_MODEL && m_model == CamModel::OV7670) {
    repModel = CamModel::ESP32_CAM;
  }
  CamRole repRole = m_role;
  if (m_fault == Esp32CamSimFault::WRONG_ROLE) {
    repRole = CamRole::US_CAM;
  }
  if (m_fault == Esp32CamSimFault::WRONG_ROLE && m_role == CamRole::US_CAM) {
    repRole = CamRole::DS_CAM;
  }
  uint8_t buf[3] = {kRspInfo, static_cast<uint8_t>(repModel), static_cast<uint8_t>(repRole)};
  push(buf, sizeof(buf));
}

void Esp32CamSimDevice::pushFrame(bool selftest) {
  const uint16_t w = selftest ? static_cast<uint16_t>(160) : static_cast<uint16_t>(640);
  const uint16_t h = selftest ? static_cast<uint16_t>(120) : static_cast<uint16_t>(480);
  const uint32_t bytes = selftest ? 24u : 1024u;
  uint8_t buf[1 + 12];
  buf[0] = kRspFrame;
  uint32_t idx = m_framesPushed;
  buf[1] = static_cast<uint8_t>(idx & 0xFF);
  buf[2] = static_cast<uint8_t>((idx >> 8) & 0xFF);
  buf[3] = static_cast<uint8_t>((idx >> 16) & 0xFF);
  buf[4] = static_cast<uint8_t>((idx >> 24) & 0xFF);
  buf[5] = static_cast<uint8_t>(bytes & 0xFF);
  buf[6] = static_cast<uint8_t>((bytes >> 8) & 0xFF);
  buf[7] = static_cast<uint8_t>((bytes >> 16) & 0xFF);
  buf[8] = static_cast<uint8_t>((bytes >> 24) & 0xFF);
  buf[9] = static_cast<uint8_t>(w & 0xFF);
  buf[10] = static_cast<uint8_t>((w >> 8) & 0xFF);
  buf[11] = static_cast<uint8_t>(h & 0xFF);
  buf[12] = static_cast<uint8_t>((h >> 8) & 0xFF);
  // ICamLink 僅推 header 通知（13 bytes），無像素
  push(buf, sizeof(buf));

  CamFrameHeader fh{};
  fh.index = idx;
  fh.bytes = bytes;
  fh.width = w;
  fh.height = h;

  if (!selftest) {
    if (m_sd != nullptr) {
      m_sd->writeFrame(fh, nullptr);
    }
    if (m_preview != nullptr) {
      CamFrameHeader preview = fh;
      preview.width = GroundPreviewSpec::kWidth;
      preview.height = GroundPreviewSpec::kHeight;
      preview.bytes = static_cast<uint32_t>(GroundPreviewSpec::kWidth) * GroundPreviewSpec::kHeight;
      m_preview->sendPreview(preview, nullptr);
    }
  }
  m_framesPushed++;
}

void Esp32CamSimDevice::advance() {
  m_tickCounter++;
  if (m_fault == Esp32CamSimFault::SILENT) {
    return;
  }
  if (m_capturing && m_fault != Esp32CamSimFault::NO_FRAMES && (m_tickCounter % 5) == 0) {
    pushFrame(false);
  }
}

uint32_t Esp32CamSimDevice::send(const uint8_t* buf, uint32_t len) {
  if (buf == nullptr || len == 0) {
    return 0;
  }
  m_sentBytes += len;
  if (m_fault == Esp32CamSimFault::SILENT) {
    return len;
  }
  for (uint32_t i = 0; i < len; ++i) {
    uint8_t c = buf[i];
    if (c == kCmdPing) {
      pushInfo();
      uint8_t pong = kRspPong;
      push(&pong, 1);
      if (m_fault != Esp32CamSimFault::NO_FRAMES) {
        pushFrame(true);
      }
    } else if (c == kCmdCapStart) {
      m_capturing = true;
    } else if (c == kCmdCapStop) {
      m_capturing = false;
    }
  }
  return len;
}

uint32_t Esp32CamSimDevice::recv(uint8_t* buf, uint32_t cap) {
  if (buf == nullptr || cap == 0) {
    return 0;
  }
  uint32_t n = 0;
  while (n < cap && m_rxHead != m_rxTail) {
    buf[n++] = m_rx[m_rxHead];
    m_rxHead = (m_rxHead + 1) % sizeof(m_rx);
  }
  return n;
}

}  // namespace Components
