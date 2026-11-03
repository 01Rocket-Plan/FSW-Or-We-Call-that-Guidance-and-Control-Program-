#include "Components/Camera/Ov3640Driver/Ov3640Driver.hpp"
#include <cstring>

namespace Components {

Ov3640Driver::Ov3640Driver(const char* compName)
    : Ov3640DriverComponentBase(compName) {
  m_core.configure(this, this, this);
}

void Ov3640Driver::configure(IOv3640FrameSource* externalFrames) {
  m_extFrames = externalFrames;
}

void Ov3640Driver::loadParams() {
  Fw::ParamValid v;
  Ov3640Params p = m_core.params();
  p.busTimeoutMisses = this->paramGet_BUS_TIMEOUT_MISSES(v);
  p.frameTimeoutTicks = this->paramGet_FRAME_TIMEOUT_TICKS(v);
  p.entriesPerTick = this->paramGet_ENTRIES_PER_TICK(v);
  p.model = this->paramGet_MODEL(v);
  p.role = this->paramGet_ROLE(v);
  p.faultRecoverK = this->paramGet_FAULT_RECOVER_K(v);
  p.faultRecoverR = this->paramGet_FAULT_RECOVER_R(v);
  p.frameMaxBytes = this->paramGet_FRAME_MAX_BYTES(v);
  m_sccbAddr = this->paramGet_SCCB_ADDR(v);
  m_frameMaxBytes = p.frameMaxBytes == 0 ? 65536u : p.frameMaxBytes;
  m_core.setParams(p);
}

void Ov3640Driver::parameterUpdated(FwPrmIdType) { loadParams(); }
void Ov3640Driver::parametersLoaded() { loadParams(); }

void Ov3640Driver::writeTlm() {
  this->tlmWrite_Status(m_core.status());
}

void Ov3640Driver::sched1Hz_handler(FwIndexType, U32) {
  m_core.tick(1u);
  writeTlm();
}

void Ov3640Driver::sched100Hz_handler(FwIndexType, U32) {
  m_core.tick(100u);
  if (++m_tlmDiv >= 100) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

void Ov3640Driver::POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.start();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov3640Driver::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.reset();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov3640Driver::STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.stop();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov3640Driver::CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  if (!m_core.captureStart()) {
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    return;
  }
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Ov3640Driver::CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.captureStop();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

bool Ov3640Driver::writeReg(uint16_t reg, uint8_t val) {
  if (!this->isConnected_sccbWrite_OutputPort(0)) return false;
  U8 data[3] = {static_cast<U8>((reg >> 8) & 0xFF), static_cast<U8>(reg & 0xFF), val};
  Fw::Buffer buf(data, sizeof(data));
  return this->sccbWrite_out(0, m_sccbAddr, buf) == Drv::I2cStatus::I2C_OK;
}

bool Ov3640Driver::readReg(uint16_t reg, uint8_t* val) {
  if (!val || !this->isConnected_sccbWriteRead_OutputPort(0)) return false;
  U8 wrb[2] = {static_cast<U8>((reg >> 8) & 0xFF), static_cast<U8>(reg & 0xFF)};
  U8 rdb[1] = {0};
  Fw::Buffer wr(wrb, sizeof(wrb));
  Fw::Buffer rd(rdb, sizeof(rdb));
  if (!(this->sccbWriteRead_out(0, m_sccbAddr, wr, rd) == Drv::I2cStatus::I2C_OK)) return false;
  *val = rdb[0];
  return true;
}

bool Ov3640Driver::pollFrame(Ov3640FrameView& out) {
  if (!m_extFrames) {
    out = Ov3640FrameView{};
    return false;
  }
  return m_extFrames->pollFrame(out);
}

bool Ov3640Driver::streamChunks(const uint8_t* data, uint32_t length, uint32_t frameNumber) {
  if (!this->isConnected_imageOut_OutputPort(0)) return false;
  if (!data || length == 0) return false;

  const U32 limit = (length < m_frameMaxBytes) ? length : m_frameMaxBytes;
  const bool truncated = length > m_frameMaxBytes;
  U32 sent = 0;
  U32 chunkIdx = 0;
  while (sent < limit) {
    U32 n = limit - sent;
    if (n > kChunk) n = kChunk;
    std::memcpy(m_imgScratch, data + sent, n);
    sent += n;
    U8 flags = 0;
    if (sent >= limit) {
      flags = static_cast<U8>(CamChunkFlag::END);
      if (truncated) {
        flags = static_cast<U8>(static_cast<U8>(CamChunkFlag::END) |
                                static_cast<U8>(CamChunkFlag::TRUNCATED));
      }
    }
    Fw::Buffer buf(m_imgScratch, n, frameNumber);
    this->imageOut_out(0, frameNumber, chunkIdx, flags, buf);
    ++chunkIdx;
  }
  if (truncated) {
    this->log_WARNING_HI_FrameTruncated(frameNumber, sent);
    return false;
  }
  return true;
}

bool Ov3640Driver::onFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) {
  return streamChunks(data, length, frameNumber);
}

void Ov3640Driver::onStateChange(CamState from, CamState to) {
  this->log_ACTIVITY_HI_StateChanged(from, to);
  writeTlm();
}

void Ov3640Driver::onFault(CamFault code, uint32_t detail) {
  this->log_WARNING_HI_FaultRaised(code, detail);
  writeTlm();
}

void Ov3640Driver::onRecoverAttempt(uint32_t attempt, uint32_t limit) {
  this->log_ACTIVITY_HI_RecoverAttempt(attempt, limit);
  writeTlm();
}

}  // namespace Components
