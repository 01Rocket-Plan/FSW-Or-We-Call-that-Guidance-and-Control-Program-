#include "Components/Camera/DsCam/Esp32CamSim.hpp"

namespace Components {

Esp32CamSim::Esp32CamSim(const char* compName) : Esp32CamSimComponentBase(compName) {
  m_dev.configure(CamModel::ESP32_CAM, CamRole::DS_CAM);
  m_core.configure(CamModel::ESP32_CAM, CamRole::DS_CAM, 3, 20);
  m_core.setLink(&m_dev);
  m_core.setSink(this);
}

void Esp32CamSim::writeTlm() { this->tlmWrite_Status(m_core.status()); }

void Esp32CamSim::respond(FwOpcodeType opCode, U32 cmdSeq, Esp32CamCmdResult r) {
  const Fw::CmdResponse resp =
      (r == Esp32CamCmdResult::OK) ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR;
  this->cmdResponse_out(opCode, cmdSeq, resp);
}

void Esp32CamSim::sched1Hz_handler(FwIndexType, U32) {
  m_core.tick1Hz();
  writeTlm();
}

void Esp32CamSim::sched100Hz_handler(FwIndexType, U32) {
  m_dev.advance();
  m_core.tick100Hz();
}

void Esp32CamSim::POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.powerOn()));
}

void Esp32CamSim::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.reset()));
}

void Esp32CamSim::CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.captureStart()));
}

void Esp32CamSim::CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.captureStop()));
}

void Esp32CamSim::SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 fault) {
  m_dev.setFault(static_cast<Esp32CamSimFault>(fault));
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Esp32CamSim::onStateChange(CamState from, CamState to) {
  this->log_ACTIVITY_HI_StateChanged(from, to);
  writeTlm();
}

void Esp32CamSim::onFault(CamFault code, uint32_t detail) {
  this->log_WARNING_HI_FaultRaised(code, detail);
  writeTlm();
}

}  // namespace Components
