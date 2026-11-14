#include "Components/Camera/Ov7670Driver/Ov7670Driver.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670Gray.hpp"
#include <cstring>

namespace Components {

Ov7670Driver::Ov7670Driver(const char* compName)
    : Ov7670DriverComponentBase(compName) {
  m_core.configure(this, this, this);
}

void Ov7670Driver::configure(IOv7670FrameSource* externalFrames) {
  m_extFrames = externalFrames;
}

void Ov7670Driver::loadParams() {
  Fw::ParamValid v;
  Ov7670Params p = m_core.params();
  p.busTimeoutMisses = this->paramGet_BUS_TIMEOUT_MISSES(v);
  p.frameTimeoutTicks = this->paramGet_FRAME_TIMEOUT_TICKS(v);
  p.entriesPerTick = this->paramGet_ENTRIES_PER_TICK(v);
  p.model = this->paramGet_MODEL(v);
  p.role = this->paramGet_ROLE(v);
  p.faultRecoverK = this->paramGet_FAULT_RECOVER_K(v);
  p.faultRecoverR = this->paramGet_FAULT_RECOVER_R(v);
  p.frameMaxBytes = this->paramGet_FRAME_MAX_BYTES(v);
  p.framePeriodTicks = this->paramGet_FRAME_PERIOD_TICKS(v);
  m_sccbAddr = this->paramGet_SCCB_ADDR(v);
  m_frameMaxBytes = p.frameMaxBytes == 0 ? Ov7670Regs::FRAME_BYTES : p.frameMaxBytes;
  m_core.setParams(p);
}

void Ov7670Driver::parameterUpdated(FwPrmIdType) { loadParams(); }
void Ov7670Driver::parametersLoaded() { loadParams(); }

void Ov7670Driver::writeTlm() {
  this->tlmWrite_Status(m_core.status());
}

void Ov7670Driver::sched1Hz_handler(FwIndexType, U32) {
  m_core.tick(1u);
  writeTlm();
}

void Ov7670Driver::sched100Hz_handler(FwIndexType, U32) {
  m_core.tick(100u);
  if (++m_tlmDiv >= 100) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

void Ov7670Driver::POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.start();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov7670Driver::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.reset();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov7670Driver::STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.stop();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  writeTlm();
}

void Ov7670Driver::CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  if (!m_core.captureStart()) {
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    return;
  }
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Ov7670Driver::CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.captureStop();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// SCCB 3 相寫：[位址+W][暫存器 8-bit][資料]
bool Ov7670Driver::writeReg(uint8_t reg, uint8_t val) {
  if (!this->isConnected_sccbWrite_OutputPort(0)) return false;
  U8 data[2] = {reg, val};
  Fw::Buffer buf(data, sizeof(data));
  return this->sccbWrite_out(0, m_sccbAddr, buf) == Drv::I2cStatus::I2C_OK;
}

// SCCB 讀：2 相寫（暫存器位址）＋ 2 相讀
bool Ov7670Driver::readReg(uint8_t reg, uint8_t* val) {
  if (!val || !this->isConnected_sccbWriteRead_OutputPort(0)) return false;
  U8 wrb[1] = {reg};
  U8 rdb[1] = {0};
  Fw::Buffer wr(wrb, sizeof(wrb));
  Fw::Buffer rd(rdb, sizeof(rdb));
  if (!(this->sccbWriteRead_out(0, m_sccbAddr, wr, rd) == Drv::I2cStatus::I2C_OK)) return false;
  *val = rdb[0];
  return true;
}

bool Ov7670Driver::pollFrame(CamFrameView& out) {
  if (!m_extFrames) {
    out = CamFrameView{};
    return false;
  }
  return m_extFrames->pollFrame(out);
}

bool Ov7670Driver::streamChunks(const uint8_t* data, uint32_t length, uint32_t frameNumber) {
  if (!this->isConnected_imageOut_OutputPort(0)) return false;
  if (!data || length == 0) return false;

  // length 是感測器的 YUYV 位元組數；送出去的是灰階（只有 Y），一像素 1 B。
  const U32 grayLen = ov7670GrayPixels(length);
  if (grayLen == 0) return false;
  const U32 limit = (grayLen < m_frameMaxBytes) ? grayLen : m_frameMaxBytes;
  const bool truncated = grayLen > m_frameMaxBytes;
  U32 sent = 0;
  U32 chunkIdx = 0;
  while (sent < limit) {
    U32 n = limit - sent;
    if (n > kChunk) n = kChunk;
    n = ov7670PackY(data, length, sent, m_imgScratch, n);
    if (n == 0) break;
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

bool Ov7670Driver::onFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) {
  return streamChunks(data, length, frameNumber);
}

void Ov7670Driver::onStateChange(CamState from, CamState to) {
  this->log_ACTIVITY_HI_StateChanged(from, to);
  writeTlm();
}

void Ov7670Driver::onFault(CamFault code, uint32_t detail) {
  this->log_WARNING_HI_FaultRaised(code, detail);
  writeTlm();
}

void Ov7670Driver::onRecoverAttempt(uint32_t attempt, uint32_t limit) {
  this->log_ACTIVITY_HI_RecoverAttempt(attempt, limit);
  writeTlm();
}

}  // namespace Components
