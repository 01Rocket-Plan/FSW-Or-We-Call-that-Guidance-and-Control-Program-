#include "Components/Camera/Ov3640Driver/Ov3640Core.hpp"

namespace Components {

namespace {

struct RegPair {
  uint16_t reg;
  uint8_t val;
};

constexpr RegPair kInitTable[] = {
    {0x3012, 0x80}, {0x3014, 0x00}, {0x3020, 0x20}, {0x3021, 0x00},
    {0x3022, 0x00}, {0x3023, 0x00}, {0x3030, 0x00}, {0x3031, 0x00},
    {0x3032, 0x00}, {0x3033, 0x00}, {0x3034, 0x00}, {0x3035, 0x00},
};

constexpr uint32_t kInitTableSize =
    static_cast<uint32_t>(sizeof(kInitTable) / sizeof(kInitTable[0]));

}  // namespace

Ov3640Core::Ov3640Core() {
  m_status.model = CamModel::OV3640;
  m_status.role = CamRole::US_CAM;
}

Ov3640Core::~Ov3640Core() = default;

void Ov3640Core::configure(IOv3640Sccb* sccb, IOv3640FrameSource* frameSrc, IOv3640Sink* sink) {
  m_sccb = sccb;
  m_frameSrc = frameSrc;
  m_sink = sink;
}

void Ov3640Core::setParams(const Ov3640Params& p) {
  m_params = p;
  m_status.model = p.model;
  m_status.role = p.role;
}

void Ov3640Core::start() {
  if (m_status.camState == CamState::OFF) {
    m_initIdx = 0;
    m_busMisses = 0;
    m_noFrameTicks = 0;
    m_idOk = false;
    m_testFrameOk = false;
    m_capturing = false;
    m_status.fault = CamFault::NONE;
    enterState(CamState::INIT);
  }
}

void Ov3640Core::stop() {
  enterState(CamState::OFF);
  m_busMisses = 0;
  m_noFrameTicks = 0;
  m_idOk = false;
  m_testFrameOk = false;
  m_capturing = false;
}

void Ov3640Core::reset() {
  m_initIdx = 0;
  m_busMisses = 0;
  m_noFrameTicks = 0;
  m_lastChipId = 0;
  m_idOk = false;
  m_testFrameOk = false;
  m_capturing = false;
  m_status.fault = CamFault::NONE;
  m_status.frameCount = 0;
  m_status.model = m_params.model;
  m_status.role = m_params.role;
  enterState(CamState::OFF);
}

bool Ov3640Core::captureStart() {
  if (m_status.camState != CamState::READY) return false;
  m_capturing = true;
  m_noFrameTicks = 0;
  return true;
}

void Ov3640Core::captureStop() {
  m_capturing = false;
  m_noFrameTicks = 0;
}

void Ov3640Core::enterState(CamState s) {
  const CamState from = m_status.camState;
  if (from == s) return;
  m_status.camState = s;
  if (m_sink) m_sink->onStateChange(from, s);
}

void Ov3640Core::enterFault(CamFault f, uint32_t detail) {
  m_status.fault = f;
  if (m_sink) m_sink->onFault(f, detail);
  enterState(CamState::FAULT);
}

void Ov3640Core::recordBus(bool ok) {
  if (ok) {
    m_busMisses = 0;
  } else {
    ++m_busMisses;
    if (m_busMisses >= m_params.busTimeoutMisses) {
      enterFault(CamFault::BUS_TIMEOUT, m_busMisses);
    }
  }
}

bool Ov3640Core::tick(uint32_t tickHz) {
  if (tickHz != 1u && tickHz != 100u) return false;

  switch (m_status.camState) {
    case CamState::OFF:
      return false;
    case CamState::INIT:
      return stepInit();
    case CamState::SELFTEST:
      return stepSelftest(tickHz);
    case CamState::READY:
      return stepReady(tickHz);
    case CamState::FAULT:
      return false;
  }
  return false;
}

bool Ov3640Core::stepInit() {
  if (!m_sccb) {
    enterFault(CamFault::BUS_TIMEOUT, 0);
    return true;
  }
  const uint32_t n = m_params.entriesPerTick;
  for (uint32_t i = 0; i < n && m_initIdx < kInitTableSize; ++i) {
    const RegPair& rp = kInitTable[m_initIdx];
    const bool ok = m_sccb->writeReg(rp.reg, rp.val);
    recordBus(ok);
    if (ok) {
      ++m_initIdx;
    } else if (m_status.camState == CamState::FAULT) {
      return true;
    }
  }
  if (m_status.camState == CamState::FAULT) return true;
  if (m_initIdx >= kInitTableSize) {
    m_idOk = false;
    m_testFrameOk = false;
    m_noFrameTicks = 0;
    enterState(CamState::SELFTEST);
  }
  return true;
}

bool Ov3640Core::stepSelftest(uint32_t tickHz) {
  if (m_params.model != kDeclaredModel || m_params.role != kDeclaredRole) {
    enterFault(CamFault::TYPE_ROLE_MISMATCH,
               (static_cast<uint32_t>(m_params.model) << 8) |
                   static_cast<uint32_t>(m_params.role));
    return true;
  }
  m_status.model = m_params.model;
  m_status.role = m_params.role;

  if (!m_sccb) {
    enterFault(CamFault::BUS_TIMEOUT, 0);
    return true;
  }

  if (!m_idOk) {
    uint8_t hi = 0, lo = 0;
    const bool ok1 = m_sccb->readReg(Ov3640Regs::PIDH, &hi);
    recordBus(ok1);
    if (m_status.camState == CamState::FAULT) return true;
    const bool ok2 = m_sccb->readReg(Ov3640Regs::PIDL, &lo);
    recordBus(ok2);
    if (m_status.camState == CamState::FAULT) return true;
    if (!ok1 || !ok2) {
      return true;
    }
    m_lastChipId = static_cast<uint16_t>((hi << 8) | lo);
    if (hi != Ov3640Regs::EXPECTED_PIDH || lo != Ov3640Regs::EXPECTED_PIDL) {
      enterFault(CamFault::ID_MISMATCH, m_lastChipId);
      return true;
    }
    m_idOk = true;
    m_noFrameTicks = 0;
  }

  if (!m_testFrameOk) {
    Ov3640FrameView view{};
    bool got = false;
    if (m_frameSrc) {
      got = m_frameSrc->pollFrame(view) && view.complete && view.length > 0;
    }
    if (got) {
      m_testFrameOk = true;
      m_noFrameTicks = 0;
      enterState(CamState::READY);
      return true;
    }
    if (tickHz == 100u) {
      ++m_noFrameTicks;
      if (m_noFrameTicks >= m_params.frameTimeoutTicks) {
        enterFault(CamFault::FRAME_TIMEOUT, m_noFrameTicks);
        return true;
      }
    }
  }
  return true;
}

bool Ov3640Core::stepReady(uint32_t tickHz) {
  if (!m_capturing) {
    return true;
  }
  Ov3640FrameView view{};
  bool got = false;
  if (m_frameSrc) {
    got = m_frameSrc->pollFrame(view) && view.complete && view.length > 0;
  }
  if (got) {
    ++m_status.frameCount;
    m_noFrameTicks = 0;
    if (m_sink) {
      m_sink->onFrame(view.data, view.length, m_status.frameCount);
    }
  } else if (tickHz == 100u) {
    ++m_noFrameTicks;
    if (m_noFrameTicks >= m_params.frameTimeoutTicks) {
      enterFault(CamFault::FRAME_TIMEOUT, m_noFrameTicks);
      return true;
    }
  }
  return true;
}

}  // namespace Components
