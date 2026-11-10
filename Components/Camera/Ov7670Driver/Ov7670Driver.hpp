// Ov7670Driver：F´ 元件外殼（SCCB→Drv.I2c；DVP 幀來源可注入；邏輯在 Ov7670Core）
#ifndef COMPONENTS_CAMERA_OV7670DRIVER_OV7670DRIVER_HPP
#define COMPONENTS_CAMERA_OV7670DRIVER_OV7670DRIVER_HPP

#include "Components/Camera/CamTypes/CamChunkFlagEnumAc.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670Core.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670DriverComponentAc.hpp"

namespace Components {

class Ov7670Driver final : public Ov7670DriverComponentBase,
                           private IOv7670Sccb,
                           private IOv7670FrameSource,
                           private IOv7670Sink {
 public:
  explicit Ov7670Driver(const char* compName);
  ~Ov7670Driver() override = default;

  void configure(IOv7670FrameSource* externalFrames);

 private:
  void sched1Hz_handler(FwIndexType portNum, U32 context) override;
  void sched100Hz_handler(FwIndexType portNum, U32 context) override;
  void POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  bool writeReg(uint8_t reg, uint8_t val) override;
  bool readReg(uint8_t reg, uint8_t* val) override;
  bool pollFrame(CamFrameView& out) override;

  bool onFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) override;
  void onStateChange(CamState from, CamState to) override;
  void onFault(CamFault code, uint32_t detail) override;
  void onRecoverAttempt(uint32_t attempt, uint32_t limit) override;

  void loadParams();
  void writeTlm();
  bool streamChunks(const uint8_t* data, uint32_t length, uint32_t frameNumber);

  Ov7670Core m_core;
  IOv7670FrameSource* m_extFrames = nullptr;
  U32 m_sccbAddr = Ov7670Regs::SCCB_ADDR_7BIT;
  U32 m_frameMaxBytes = Ov7670Regs::FRAME_BYTES;
  U32 m_tlmDiv = 0;
  // Only one chunk buffer — never hold a full frame in RAM.
  static constexpr U32 kChunk = 2048;
  U8 m_imgScratch[kChunk]{};
};

}  // namespace Components
#endif
