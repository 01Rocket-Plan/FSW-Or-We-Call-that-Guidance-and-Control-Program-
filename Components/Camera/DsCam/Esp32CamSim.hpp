// Esp32CamSim：同一 Esp32CamCore + Esp32CamSimDevice
#ifndef COMPONENTS_CAMERA_DSCAM_ESP32CAMSIM_HPP
#define COMPONENTS_CAMERA_DSCAM_ESP32CAMSIM_HPP

#include "Components/Camera/DsCam/Esp32CamCmd.hpp"
#include "Components/Camera/DsCam/Esp32CamCore.hpp"
#include "Components/Camera/DsCam/Esp32CamSimComponentAc.hpp"
#include "Components/Camera/DsCam/Esp32CamSimDevice.hpp"

namespace Components {

class Esp32CamSim final : public Esp32CamSimComponentBase, private ICamSink {
 public:
  explicit Esp32CamSim(const char* compName);
  ~Esp32CamSim() override = default;

 private:
  void sched1Hz_handler(FwIndexType portNum, U32 context) override;
  void sched100Hz_handler(FwIndexType portNum, U32 context) override;

  void POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 fault) override;

  void onStateChange(CamState from, CamState to) override;
  void onFault(CamFault code, uint32_t detail) override;

  void writeTlm();
  void respond(FwOpcodeType opCode, U32 cmdSeq, Esp32CamCmdResult r);

  Esp32CamSimDevice m_dev;
  Esp32CamCore m_core;
};

}  // namespace Components
#endif
