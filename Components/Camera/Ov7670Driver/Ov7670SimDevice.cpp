#include "Components/Camera/Ov7670Driver/Ov7670SimDevice.hpp"

namespace Components {

Ov7670SimDevice::Ov7670SimDevice() {
  for (auto& b : m_regs) b = 0;
  m_regs[Ov7670Regs::PID] = Ov7670Regs::EXPECTED_PID;  // 唯讀 ID 暫存器
  m_regs[Ov7670Regs::VER] = Ov7670Regs::EXPECTED_VER;
  m_regs[Ov7670Regs::MIDH] = 0x7F;
  m_regs[Ov7670Regs::MIDL] = 0xA2;
  for (uint32_t i = 0; i < kSimFrameCap; ++i) {
    m_frameBuf[i] = static_cast<uint8_t>(0xA0 + (i & 0x1Fu));
  }
}

void Ov7670SimDevice::setReportedFrameLength(uint32_t n) {
  m_reportedLen = (n > kSimFrameCap) ? kSimFrameCap : n;
  if (m_reportedLen == 0) m_reportedLen = kSimFrameBytes;
}

void Ov7670SimDevice::advance() {
  ++m_frameTick;
  if (m_frameEveryN > 0) {
    m_frameAvail = ((m_frameTick % m_frameEveryN) == 0);
  } else {
    m_frameAvail = true;
  }
}

void Ov7670SimDevice::setFrameEveryN(uint32_t n) {
  m_frameEveryN = n;
  m_frameTick = 0;
  m_frameAvail = (n == 0);
}

void Ov7670SimDevice::regWrite(uint8_t reg, uint8_t val) {
  // ID 暫存器唯讀（真晶片寫了也不會變）
  if (reg == Ov7670Regs::PID || reg == Ov7670Regs::VER || reg == Ov7670Regs::MIDH ||
      reg == Ov7670Regs::MIDL) {
    return;
  }
  m_regs[reg] = val;
}

bool Ov7670SimDevice::writeReg(uint8_t reg, uint8_t val) {
  ++m_writeCount;
  switch (m_fault) {
    case Ov7670SimFault::NACK_ON_WRITE:
    case Ov7670SimFault::BUS_DOWN:
      ++m_writeFail;
      return false;
    default:
      break;
  }
  regWrite(reg, val);
  return true;
}

bool Ov7670SimDevice::readReg(uint8_t reg, uint8_t* val) {
  ++m_readCount;
  if (val == nullptr) return false;
  switch (m_fault) {
    case Ov7670SimFault::NACK_ON_READ:
    case Ov7670SimFault::BUS_DOWN:
      ++m_readFail;
      return false;
    case Ov7670SimFault::STUCK_READ:
      *val = m_stuck;
      return true;
    case Ov7670SimFault::BAD_CHIP_ID:
      if (reg == Ov7670Regs::PID || reg == Ov7670Regs::VER) {
        *val = 0x00;
        return true;
      }
      *val = m_regs[reg];
      return true;
    default:
      break;
  }
  *val = m_regs[reg];
  return true;
}

bool Ov7670SimDevice::pollFrame(CamFrameView& out) {
  if (!m_frameAvail) {
    out = CamFrameView{};
    return false;
  }
  out.data = m_frameBuf;
  out.length = m_reportedLen;
  out.complete = true;
  if (m_frameEveryN > 0) {
    m_frameAvail = false;
  }
  return true;
}

}  // namespace Components
