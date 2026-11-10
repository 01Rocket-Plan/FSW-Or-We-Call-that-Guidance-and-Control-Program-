#ifndef COMPONENTS_CAMERA_OV7670DRIVER_OV7670DRIVERSIM_HPP
#define COMPONENTS_CAMERA_OV7670DRIVER_OV7670DRIVERSIM_HPP

#include "Components/Camera/CamTypes/CamChunkFlagEnumAc.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670Core.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670DriverSimComponentAc.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670SimDevice.hpp"

namespace Components {

class Ov7670DriverSim final : public Ov7670DriverSimComponentBase, private IOv7670Sink {
 public:
  explicit Ov7670DriverSim(const char* compName);
  ~Ov7670DriverSim() override = default;

  //! SIL 拓樸用：開機自動上電／開始擷取，模擬 fps（100 Hz 下每 N 拍一幀；0＝每拍）、幀大小。
  //! 真機的上電／擷取由 CamLink 指令觸發；SIL 沒有地面站時用這個讓相機自己跑起來。
  void configureSil(bool autoPowerOn, bool autoCapture, U32 frameEveryN, U32 frameBytes);

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

  bool onFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) override;
  void onStateChange(CamState from, CamState to) override;
  void onFault(CamFault code, uint32_t detail) override;
  void onRecoverAttempt(uint32_t attempt, uint32_t limit) override;

  void loadParams();
  void writeTlm();
  bool streamChunks(const uint8_t* data, uint32_t length, uint32_t frameNumber);

  Ov7670Core m_core;
  Ov7670SimDevice m_dev;
  U32 m_frameMaxBytes = Ov7670Regs::FRAME_BYTES;
  bool m_autoPowerOn = false;
  bool m_autoCapture = false;
  U32 m_tlmDiv = 0;
  static constexpr U32 kChunk = 2048;
  U8 m_imgScratch[kChunk]{};
};

}  // namespace Components
#endif
