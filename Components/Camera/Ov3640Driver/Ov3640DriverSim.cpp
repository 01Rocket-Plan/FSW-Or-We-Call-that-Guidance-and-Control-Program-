#include "Components/Camera/Ov3640Driver/Ov3640DriverSim.hpp"
#include <cstring>

namespace Components {

Ov3640DriverSim::Ov3640DriverSim(const char* compName)
    : Ov3640DriverSimComponentBase(compName) {
  m_core.configure(&m_dev, &m_dev, this);
}

void Ov3640DriverSim::loadParams() {
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
  m_frameMaxBytes = p.frameMaxBytes == 0 ? 65536u : p.frameMaxBytes;
  m_core.setParams(p);
}

void Ov3640DriverSim::parameterUpdated(FwPrmIdType) { loadParams(); }
void Ov3640DriverSim::parametersLoaded() { loadParams(); }

void Ov3640DriverSim::writeTlm() { this->tlmWrite_Status(m_core.status()); }

void Ov3640DriverSim::sched1Hz_handler(FwIndexType, U32) {
  m_dev.advance();
  m_core.tick(1u);
  writeTlm();
}

void Ov3640DriverSim::sched100Hz_handler(FwIndexType, U32) {
  m_dev.advance();
  m_core.tick(100u);
  if (++m_tlmDiv >= 100) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

void Ov3640DriverSim::POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.start();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov3640DriverSim::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.reset();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov3640DriverSim::STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.stop();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov3640DriverSim::CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  if (!m_core.captureStart()) {
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    return;
  }
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Ov3640DriverSim::CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.captureStop();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Ov3640DriverSim::SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 fault) {
  m_dev.setFault(static_cast<Ov3640SimFault>(fault));
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Ov3640DriverSim::SIM_SET_FRAME_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 enable) {
  m_dev.setFrameAvailable(enable != 0);
  if (enable == 0) m_dev.setFrameEveryN(0xFFFFFFFFu);
  else m_dev.setFrameEveryN(0);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

bool Ov3640DriverSim::streamChunks(const uint8_t* data, uint32_t length, uint32_t frameNumber) {
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

bool Ov3640DriverSim::onFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) {
  return streamChunks(data, length, frameNumber);
}

void Ov3640DriverSim::onStateChange(CamState from, CamState to) {
  this->log_ACTIVITY_HI_StateChanged(from, to);
  writeTlm();
}

void Ov3640DriverSim::onFault(CamFault code, uint32_t detail) {
  this->log_WARNING_HI_FaultRaised(code, detail);
  writeTlm();
}

void Ov3640DriverSim::onRecoverAttempt(uint32_t attempt, uint32_t limit) {
  this->log_ACTIVITY_HI_RecoverAttempt(attempt, limit);
  writeTlm();
}

}  // namespace Components
