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

void Ov3640DriverSim::onFrame(const uint8_t* data, uint32_t length, uint32_t frameIndex) {
  if (!this->isConnected_imageOut_OutputPort(0)) return;
  if (!data || length == 0) return;
  const U32 n = (length < kImgCap) ? length : kImgCap;
  std::memcpy(m_imgScratch, data, n);
  Fw::Buffer buf(m_imgScratch, n, frameIndex);
  this->imageOut_out(0, frameIndex, buf);
}

void Ov3640DriverSim::onStateChange(CamState from, CamState to) {
  this->log_ACTIVITY_HI_StateChanged(from, to);
  writeTlm();
}

void Ov3640DriverSim::onFault(CamFault code, uint32_t detail) {
  this->log_WARNING_HI_FaultRaised(code, detail);
  writeTlm();
}

}  // namespace Components
