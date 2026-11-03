// Ov3640Driver：F´ 元件外殼（SCCB→Drv.I2c；DVP 幀來源可注入；邏輯在 Ov3640Core）
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_OV3640DRIVER_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_OV3640DRIVER_HPP

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

  // 注入外部 DVP 幀來源（不取得所有權）。未注入時 pollFrame 回 false。
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

  // IOv3640Sccb — 經 sccbWrite / sccbWriteRead 埠
  bool writeReg(uint16_t reg, uint8_t val) override;
  bool readReg(uint16_t reg, uint8_t* val) override;

  // IOv3640FrameSource — 轉呼叫外部來源
  bool pollFrame(Ov3640FrameView& out) override;

  // IOv3640Sink
  void onFrame(const uint8_t* data, uint32_t length, uint32_t frameIndex) override;
  void onStateChange(CamState from, CamState to) override;
  void onFault(CamFault code, uint32_t detail) override;

  void loadParams();
  void writeTlm();

  Ov3640Core m_core;
  IOv3640FrameSource* m_extFrames = nullptr;
  U32 m_sccbAddr = 0x3C;
  U32 m_tlmDiv = 0;
  // 靜態幀暫存（無動態配置）：imageOut 送出前拷貝長度上限
  static constexpr U32 kImgCap = 2048;
  U8 m_imgScratch[kImgCap]{};
};

}  // namespace Components
#endif
