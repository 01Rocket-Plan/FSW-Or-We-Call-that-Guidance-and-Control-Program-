// Ov3640Driver：F´ 元件外殼（SCCB→Drv.I2c；DVP 幀來源可注入；邏輯在 Ov3640Core）
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_OV3640DRIVER_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_OV3640DRIVER_HPP

#include "Components/Camera/CamTypes/CamChunkFlagEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640Core.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640DriverComponentAc.hpp"

namespace Components {

class Ov3640Driver final : public Ov3640DriverComponentBase,
                           private IOv3640Sccb,
                           private IOv3640FrameSource,
                           private IOv3640Sink {
 public:
  explicit Ov3640Driver(const char* compName);
  ~Ov3640Driver() override = default;

  void configure(IOv3640FrameSource* externalFrames);

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

  bool writeReg(uint16_t reg, uint8_t val) override;
  bool readReg(uint16_t reg, uint8_t* val) override;
  bool pollFrame(CamFrameView& out) override;

  bool onFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) override;
  void onStateChange(CamState from, CamState to) override;
  void onFault(CamFault code, uint32_t detail) override;
  void onRecoverAttempt(uint32_t attempt, uint32_t limit) override;

  void loadParams();
  void writeTlm();
  bool streamChunks(const uint8_t* data, uint32_t length, uint32_t frameNumber);

  Ov3640Core m_core;
  IOv3640FrameSource* m_extFrames = nullptr;
  U32 m_sccbAddr = 0x3C;
  U32 m_frameMaxBytes = 65536;
  U32 m_tlmDiv = 0;
  // Only one chunk buffer — never hold a full JPEG in RAM.
  static constexpr U32 kChunk = 2048;
  U8 m_imgScratch[kChunk]{};
};

}  // namespace Components
#endif
