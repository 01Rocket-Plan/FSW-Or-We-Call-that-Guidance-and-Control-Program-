// Ov3640DriverSim：同一 Core，接 Ov3640SimDevice（host／拓樸 Sim）
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_OV3640DRIVERSIM_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_OV3640DRIVERSIM_HPP

#include "Components/Camera/Ov3640Driver/Ov3640Core.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640DriverSimComponentAc.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640SimDevice.hpp"

namespace Components {

class Ov3640DriverSim final : public Ov3640DriverSimComponentBase, private IOv3640Sink {
 public:
  explicit Ov3640DriverSim(const char* compName);
  ~Ov3640DriverSim() override = default;

 private:
  void sched1Hz_handler(FwIndexType portNum, U32 context) override;
  void sched100Hz_handler(FwIndexType portNum, U32 context) override;
  void POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 fault) override;
  void SIM_SET_FRAME_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 enable) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void onFrame(const uint8_t* data, uint32_t length, uint32_t frameIndex) override;
  void onStateChange(CamState from, CamState to) override;
  void onFault(CamFault code, uint32_t detail) override;

  void loadParams();
  void writeTlm();

  Ov3640SimDevice m_dev;
  Ov3640Core m_core;
  U32 m_tlmDiv = 0;
  static constexpr U32 kImgCap = 2048;
  U8 m_imgScratch[kImgCap]{};
};

}  // namespace Components
#endif
