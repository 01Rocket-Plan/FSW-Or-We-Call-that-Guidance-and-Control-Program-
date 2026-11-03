#include "Components/Camera/Ov3640Driver/Ov3640SimDevice.hpp"

namespace Components {

Ov3640SimDevice::Ov3640SimDevice() {
  for (auto& b : m_regsLo) b = 0;
  m_regsLo[Ov3640Regs::PIDH & 0xFF] = m_pidh;
  m_regsLo[Ov3640Regs::PIDL & 0xFF] = m_pidl;
  for (uint32_t i = 0; i < kSimFrameBytes; ++i) {
    m_frameBuf[i] = static_cast<uint8_t>(0xA0 + i);
  }
}

void Ov3640SimDevice::advance() {
  ++m_frameTick;
  if (m_frameEveryN > 0) {
    m_frameAvail = ((m_frameTick % m_frameEveryN) == 0);
  } else {
    m_frameAvail = true;
  }
}

void Ov3640SimDevice::setFrameEveryN(uint32_t n) {
  m_frameEveryN = n;
  m_frameTick = 0;
  m_frameAvail = (n == 0);
}

uint8_t Ov3640SimDevice::reg8(uint16_t reg) const {
  return m_regsLo[reg & 0xFF];
}

void Ov3640SimDevice::regWrite(uint16_t reg, uint8_t val) {
  m_regsLo[reg & 0xFF] = val;
  if (reg == Ov3640Regs::PIDH) m_pidh = val;
  if (reg == Ov3640Regs::PIDL) m_pidl = val;
}

bool Ov3640SimDevice::writeReg(uint16_t reg, uint8_t val) {
  ++m_writeCount;
  switch (m_fault) {
    case Ov3640SimFault::NACK_ON_WRITE:
    case Ov3640SimFault::BUS_DOWN:
      ++m_writeFail;
      return false;
    default:
      break;
  }
  regWrite(reg, val);
  return true;
}

bool Ov3640SimDevice::readReg(uint16_t reg, uint8_t* val) {
  ++m_readCount;
  if (val == nullptr) return false;
  switch (m_fault) {
    case Ov3640SimFault::NACK_ON_READ:
    case Ov3640SimFault::BUS_DOWN:
      ++m_readFail;
      return false;
    case Ov3640SimFault::STUCK_READ:
      *val = m_stuck;
      return true;
    case Ov3640SimFault::BAD_CHIP_ID:
      if (reg == Ov3640Regs::PIDH || reg == Ov3640Regs::PIDL) {
        *val = 0x00;
        return true;
      }
      *val = reg8(reg);
      return true;
    default:
      break;
  }
  *val = reg8(reg);
  return true;
}

bool Ov3640SimDevice::pollFrame(Ov3640FrameView& out) {
  if (!m_frameAvail) {
    out = Ov3640FrameView{};
    return false;
  }
  out.data = m_frameBuf;
  out.length = kSimFrameBytes;
  out.complete = true;
  if (m_frameEveryN > 0) {
    m_frameAvail = false;
  }
  return true;
}

}  // namespace Components
