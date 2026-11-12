// Esp32CamCore 實作：非阻塞、有界工作、每 tick 固定量。
#include "Components/Camera/DsCam/Esp32CamCore.hpp"

namespace Components {

namespace {
constexpr uint8_t kCmdPing = 0xA1;
constexpr uint8_t kRspPong = 0xA2;
constexpr uint8_t kCmdCapStart = 0xB1;
constexpr uint8_t kCmdCapStop = 0xB2;
constexpr uint8_t kRspFrame = 0xC0;  // 幀通知（僅 12-byte header，無像素）
constexpr uint8_t kRspInfo = 0xD0;   // model(1) role(1)
}  // namespace

Esp32CamCore::Esp32CamCore() {
  m_i2c.clockHz = kDefaultI2cClockHz;
}

void Esp32CamCore::configure(CamModel model, CamRole role, uint8_t missLimit, uint32_t frameLimit) {
  m_model = model;
  m_role = role;
  m_missLimit = (missLimit == 0) ? kDefaultMissLimit : missLimit;
  m_frameLimit = (frameLimit == 0) ? kDefaultFrameLimit : frameLimit;
}

void Esp32CamCore::setI2cClockHz(uint32_t hz) {
  m_i2c.clockHz = (hz == 0) ? kDefaultI2cClockHz : hz;
}

void Esp32CamCore::enter(CamState s) {
  if (s == m_state) {
    return;
  }
  CamState from = m_state;
  m_state = s;
  if (m_sink != nullptr) {
    m_sink->onStateChange(from, s);
  }
}

void Esp32CamCore::enterFault(CamFault f, uint32_t detail) {
  // FRAME_TRUNCATED：僅事件／紀錄，不進 FAULT、不停擷取（對齊 Bot2 PR#12／Edward）
  if (f == CamFault::FRAME_TRUNCATED) {
    if (m_sink != nullptr) {
      m_sink->onFault(f, detail);
    }
    return;
  }
  if (m_state == CamState::FAULT) {
    return;
  }
  m_fault = f;
  m_captureOn = false;
  enter(CamState::FAULT);
  if (m_sink != nullptr) {
    m_sink->onFault(f, detail);
  }
}

CamStatus Esp32CamCore::status() const {
  CamStatus s{};
  s.set_camState(m_state);
  s.set_fault(m_fault);
  s.set_frameCount(m_frameCount);
  s.set_model(m_model);
  s.set_role(m_role);
  return s;
}

bool Esp32CamCore::powerOn() {
  if (m_state != CamState::OFF) {
    return false;
  }
  m_fault = CamFault::NONE;
  m_missCount = 0;
  m_noFrameTicks = 0;
  m_selftestMsgSeen = false;
  m_selftestFrameSeen = false;
  m_captureOn = false;
  enter(CamState::INIT);
  return true;
}

bool Esp32CamCore::reset() {
  m_frameCount = 0;
  m_fault = CamFault::NONE;
  m_missCount = 0;
  m_noFrameTicks = 0;
  m_selftestMsgSeen = false;
  m_selftestFrameSeen = false;
  m_captureOn = false;
  enter(CamState::OFF);
  return true;
}

bool Esp32CamCore::captureStart() {
  if (m_state != CamState::READY) {
    return false;
  }
  m_captureOn = true;
  m_noFrameTicks = 0;
  if (m_link != nullptr) {
    uint8_t cmd = kCmdCapStart;
    m_link->send(&cmd, 1);
  }
  return true;
}

bool Esp32CamCore::captureStop() {
  if (m_state != CamState::READY) {
    return false;
  }
  m_captureOn = false;
  m_noFrameTicks = 0;
  if (m_link != nullptr) {
    uint8_t cmd = kCmdCapStop;
    m_link->send(&cmd, 1);
  }
  return true;
}

void Esp32CamCore::notifyFrameTruncated(uint32_t length) {
  enterFault(CamFault::FRAME_TRUNCATED, length);
}

bool Esp32CamCore::ingestCapturedFrame(const uint8_t* data, uint32_t length) {
  if (m_state != CamState::READY || !m_captureOn) {
    return false;
  }
  if (data == nullptr || length == 0) {
    return false;
  }
  const uint32_t frameNumber = m_frameCount + 1u;
  const bool accepted = m_streamer.streamFrame(data, length, frameNumber);
  if (!accepted) {
    // 截斷：事件 only，維持 READY／擷取中，不計 frameCount
    notifyFrameTruncated(length);
    return false;
  }
  m_frameCount++;
  m_noFrameTicks = 0;
  return true;
}

void Esp32CamCore::sendHandshake() {
  if (m_link == nullptr) {
    return;
  }
  uint8_t cmd = kCmdPing;
  m_link->send(&cmd, 1);
}

bool Esp32CamCore::pullByte(uint8_t* out) {
  if (m_link == nullptr || out == nullptr) {
    return false;
  }
  return m_link->recv(out, 1) == 1;
}

bool Esp32CamCore::consumeFrameNotify() {
  uint8_t hdr[12];
  if (m_link == nullptr || m_link->recv(hdr, 12) != 12) {
    return false;
  }
  return true;
}

void Esp32CamCore::checkSelftest() {
  uint8_t b = 0;
  if (!pullByte(&b)) {
    if (m_missCount < 255) {
      m_missCount++;
    }
    if (m_missCount >= m_missLimit) {
      enterFault(CamFault::BUS_TIMEOUT, m_missCount);
      return;
    }
    sendHandshake();
    return;
  }
  m_missCount = 0;

  if (b == kRspInfo) {
    uint8_t info[2] = {0, 0};
    uint32_t got = (m_link == nullptr) ? 0 : m_link->recv(info, 2);
    if (got == 2) {
      CamModel repModel = static_cast<CamModel>(info[0]);
      CamRole repRole = static_cast<CamRole>(info[1]);
      if (repModel != m_model || repRole != m_role) {
        enterFault(CamFault::TYPE_ROLE_MISMATCH, 0);
        return;
      }
      m_selftestMsgSeen = true;
    }
  } else if (b == kRspFrame) {
    if (consumeFrameNotify()) {
      m_selftestFrameSeen = true;
      m_noFrameTicks = 0;
    }
  } else if (b == kRspPong) {
    // ok
  }

  if (m_selftestMsgSeen && m_selftestFrameSeen) {
    enter(CamState::READY);
    return;
  }
  sendHandshake();
}

void Esp32CamCore::tick1Hz() {
  switch (m_state) {
    case CamState::OFF:
      break;
    case CamState::INIT:
      m_missCount = 0;
      m_selftestMsgSeen = false;
      m_selftestFrameSeen = false;
      m_noFrameTicks = 0;
      enter(CamState::SELFTEST);
      sendHandshake();
      break;
    case CamState::SELFTEST:
      checkSelftest();
      break;
    case CamState::READY:
      break;
    case CamState::FAULT:
      break;
  }
}

void Esp32CamCore::tick100Hz() {
  if (m_state == CamState::SELFTEST) {
    if (!m_selftestFrameSeen) {
      if (++m_noFrameTicks >= m_frameLimit) {
        enterFault(CamFault::FRAME_TIMEOUT, m_noFrameTicks);
        return;
      }
    }
    // 有界：順便消化一則回應，加快自檢
    uint8_t b = 0;
    if (!pullByte(&b)) {
      return;
    }
    m_missCount = 0;
    if (b == kRspFrame) {
      if (consumeFrameNotify()) {
        m_selftestFrameSeen = true;
        m_noFrameTicks = 0;
      }
    } else if (b == kRspInfo) {
      uint8_t info[2];
      if (m_link != nullptr && m_link->recv(info, 2) == 2) {
        CamModel repModel = static_cast<CamModel>(info[0]);
        CamRole repRole = static_cast<CamRole>(info[1]);
        if (repModel != m_model || repRole != m_role) {
          enterFault(CamFault::TYPE_ROLE_MISMATCH, 0);
          return;
        }
        m_selftestMsgSeen = true;
      }
    }
    if (m_selftestMsgSeen && m_selftestFrameSeen) {
      enter(CamState::READY);
    }
    return;
  }

  if (m_state != CamState::READY || !m_captureOn) {
    return;
  }

  uint8_t b = 0;
  bool gotFrame = false;
  if (pullByte(&b) && b == kRspFrame) {
    if (consumeFrameNotify()) {
      m_frameCount++;
      gotFrame = true;
    }
  }

  if (gotFrame) {
    m_noFrameTicks = 0;
  } else if (++m_noFrameTicks >= m_frameLimit) {
    enterFault(CamFault::FRAME_TIMEOUT, m_noFrameTicks);
  }
}

}  // namespace Components
