#include "Components/Camera/Ov7670Driver/Ov7670Core.hpp"

namespace Components {

namespace {

struct RegPair {
  uint8_t reg;
  uint8_t val;
};

// 第 0 筆是軟重置（COM7=0x80）；寫完這一拍就結束，下一拍（100 Hz＝10 ms 後，> 規格 1 ms）再寫其餘。
// 輸出：QVGA 320×240、RGB565。值對照 Linux ov7670.c（ov7670_default_regs／QVGA／RGB565 段）。
constexpr RegPair kInitTable[] = {
    {Ov7670Regs::COM7, Ov7670Regs::COM7_RESET},
    {Ov7670Regs::CLKRC, 0x01},
    {Ov7670Regs::COM7, Ov7670Regs::COM7_QVGA_RGB},
    {Ov7670Regs::RGB444, 0x00},
    {Ov7670Regs::COM15, 0xD0},
    {Ov7670Regs::TSLB, 0x04},
    {Ov7670Regs::COM3, 0x04},
    {Ov7670Regs::COM14, 0x19},
    {Ov7670Regs::SCALING_XSC, 0x3A},
    {Ov7670Regs::SCALING_YSC, 0x35},
    {Ov7670Regs::SCALING_DCWCTR, 0x11},
    {Ov7670Regs::SCALING_PCLK_DIV, 0xF1},
    {Ov7670Regs::SCALING_PCLK_DELAY, 0x02},
};

constexpr uint32_t kInitTableSize =
    static_cast<uint32_t>(sizeof(kInitTable) / sizeof(kInitTable[0]));

}  // namespace

Ov7670Core::Ov7670Core() = default;

Ov7670Core::~Ov7670Core() = default;

void Ov7670Core::configure(IOv7670Sccb* sccb, IOv7670FrameSource* frameSrc, IOv7670Sink* sink) {
  m_sccb = sccb;
  m_frameSrc = frameSrc;
  m_sink = sink;
}

void Ov7670Core::setParams(const Ov7670Params& p) {
  m_params = p;
  m_model = p.model;
  m_role = p.role;
}

CamStatus Ov7670Core::status() const {
  return CamStatus(m_state, m_fault, m_frameCount, m_model, m_role);
}

void Ov7670Core::beginInitHandshake() {
  m_initIdx = 0;
  m_busMisses = 0;
  m_noFrameTicks = 0;
  m_idOk = false;
  m_testFrameOk = false;
  m_capturing = false;
  m_fault = CamFault::NONE;
  enterState(CamState::INIT);
}

void Ov7670Core::start() {
  if (m_state == CamState::OFF) {
    beginInitHandshake();
  }
}

void Ov7670Core::stop() {
  enterState(CamState::OFF);
  m_busMisses = 0;
  m_noFrameTicks = 0;
  m_idOk = false;
  m_testFrameOk = false;
  m_capturing = false;
}

void Ov7670Core::reset() {
  m_initIdx = 0;
  m_busMisses = 0;
  m_noFrameTicks = 0;
  m_lastChipId = 0;
  m_idOk = false;
  m_testFrameOk = false;
  m_capturing = false;
  m_faultTicks = 0;
  m_recoverAttempts = 0;
  m_recoverLatched = false;
  m_fault = CamFault::NONE;
  m_frameCount = 0;
  m_model = m_params.model;
  m_role = m_params.role;
  enterState(CamState::OFF);
}

bool Ov7670Core::captureStart() {
  if (m_state != CamState::READY) return false;
  m_capturing = true;
  m_noFrameTicks = 0;
  return true;
}

void Ov7670Core::captureStop() {
  m_capturing = false;
  m_noFrameTicks = 0;
}

void Ov7670Core::enterState(CamState::T s) {
  const CamState from = m_state;
  if (from == s) return;
  m_state = s;
  if (m_sink) m_sink->onStateChange(from, m_state);
}

void Ov7670Core::enterFault(CamFault::T f, uint32_t detail) {
  m_fault = f;
  m_faultTicks = 0;
  m_capturing = false;
  if (m_sink) m_sink->onFault(f, detail);
  enterState(CamState::FAULT);
}

void Ov7670Core::recordBus(bool ok) {
  if (ok) {
    m_busMisses = 0;
  } else {
    ++m_busMisses;
    if (m_busMisses >= m_params.busTimeoutMisses) {
      enterFault(CamFault::BUS_TIMEOUT, m_busMisses);
    }
  }
}

bool Ov7670Core::tick(uint32_t tickHz) {
  if (tickHz != 1u && tickHz != 100u) return false;

  switch (m_state.e) {
    case CamState::OFF:
      return false;
    case CamState::INIT:
      return stepInit();
    case CamState::SELFTEST:
      return stepSelftest(tickHz);
    case CamState::READY:
      return stepReady(tickHz);
    case CamState::FAULT:
      return stepFault(tickHz);
  }
  return false;
}

bool Ov7670Core::stepInit() {
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
      if (m_initIdx == 1u) {
        break;  // 軟重置後等下一拍（規格：重置後 1 ms 內不得存取）
      }
    } else if (m_state == CamState::FAULT) {
      return true;
    }
  }
  if (m_state == CamState::FAULT) return true;
  if (m_initIdx >= kInitTableSize) {
    m_idOk = false;
    m_testFrameOk = false;
    m_noFrameTicks = 0;
    enterState(CamState::SELFTEST);
  }
  return true;
}

bool Ov7670Core::stepSelftest(uint32_t tickHz) {
  if (m_params.model != kDeclaredModel || m_params.role != kDeclaredRole) {
    enterFault(CamFault::TYPE_ROLE_MISMATCH,
               (static_cast<uint32_t>(m_params.model.e) << 8) |
                   static_cast<uint32_t>(m_params.role.e));
    return true;
  }
  m_model = m_params.model;
  m_role = m_params.role;

  if (!m_sccb) {
    enterFault(CamFault::BUS_TIMEOUT, 0);
    return true;
  }

  if (!m_idOk) {
    uint8_t hi = 0, lo = 0;
    const bool ok1 = m_sccb->readReg(Ov7670Regs::PID, &hi);
    recordBus(ok1);
    if (m_state == CamState::FAULT) return true;
    const bool ok2 = m_sccb->readReg(Ov7670Regs::VER, &lo);
    recordBus(ok2);
    if (m_state == CamState::FAULT) return true;
    if (!ok1 || !ok2) {
      return true;
    }
    m_lastChipId = static_cast<uint16_t>((hi << 8) | lo);
    if (hi != Ov7670Regs::EXPECTED_PID || lo != Ov7670Regs::EXPECTED_VER) {
      enterFault(CamFault::ID_MISMATCH, m_lastChipId);
      return true;
    }
    m_idOk = true;
    m_noFrameTicks = 0;
  }

  if (!m_testFrameOk) {
    CamFrameView view{};
    bool got = false;
    if (m_frameSrc) {
      got = m_frameSrc->pollFrame(view) && view.complete && view.length > 0;
    }
    if (got) {
      m_testFrameOk = true;
      m_noFrameTicks = 0;
      m_recoverAttempts = 0;
      m_recoverLatched = false;
      m_faultTicks = 0;
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

bool Ov7670Core::stepReady(uint32_t tickHz) {
  if (!m_capturing) {
    return true;
  }
  CamFrameView view{};
  bool got = false;
  if (m_frameSrc) {
    got = m_frameSrc->pollFrame(view) && view.complete && view.length > 0;
  }
  if (got) {
    m_noFrameTicks = 0;
    const uint32_t frameNumber = m_frameCount + 1u;
    bool accepted = true;
    if (m_sink) {
      accepted = m_sink->onFrame(view.data, view.length, frameNumber);
    }
    if (accepted) {
      ++m_frameCount;
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

bool Ov7670Core::stepFault(uint32_t tickHz) {
  // After R failed retries, latch — only RESET clears.
  if (m_recoverLatched) {
    return false;
  }
  if (tickHz != 1u) {
    return false;
  }
  const uint32_t K = m_params.faultRecoverK == 0 ? 1u : m_params.faultRecoverK;
  const uint32_t R = m_params.faultRecoverR;
  ++m_faultTicks;
  if (m_faultTicks < K) {
    return false;
  }
  m_faultTicks = 0;
  if (m_recoverAttempts >= R) {
    m_recoverLatched = true;
    return false;
  }
  ++m_recoverAttempts;
  if (m_sink) {
    m_sink->onRecoverAttempt(m_recoverAttempts, R);
  }
  beginInitHandshake();
  return true;
}

}  // namespace Components
