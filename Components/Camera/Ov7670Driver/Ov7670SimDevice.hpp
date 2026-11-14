// Ov7670SimDevice：OV7670 SCCB（8-bit 位址）／DVP 模擬（與 Core 共用同一組介面）
#ifndef COMPONENTS_CAMERA_OV7670DRIVER_OV7670SIMDEVICE_HPP
#define COMPONENTS_CAMERA_OV7670DRIVER_OV7670SIMDEVICE_HPP

#include "Components/Camera/Ov7670Driver/Ov7670Core.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670Regs.hpp"
#include <cstdint>

namespace Components {

enum class Ov7670SimFault : uint8_t {
  NONE = 0,
  NACK_ON_WRITE = 1,
  NACK_ON_READ = 2,
  BUS_DOWN = 3,
  BAD_CHIP_ID = 4,
  STUCK_READ = 5
};

class Ov7670SimDevice : public IOv7670Sccb, public IOv7670FrameSource {
 public:
  static constexpr uint32_t kSimFrameBytes = 16;
  // 最大可模擬感測器一整張 QCIF YUYV（50,688 B）；SIL／host 測試用，不是飛行 RAM 配置。
  static constexpr uint32_t kSimFrameCap = Ov7670Regs::RAW_FRAME_BYTES;

  Ov7670SimDevice();

  void advance();
  void setFault(Ov7670SimFault f) { m_fault = f; }
  void setStuckValue(uint8_t v) { m_stuck = v; }
  void setFrameAvailable(bool avail) { m_frameAvail = avail; }
  void setFrameEveryN(uint32_t n);
  // Host-test only: report length up to kSimFrameCap (multi-chunk / truncation).
  void setReportedFrameLength(uint32_t n);

  bool writeReg(uint8_t reg, uint8_t val) override;
  bool readReg(uint8_t reg, uint8_t* val) override;
  bool pollFrame(CamFrameView& out) override;

  uint32_t writeCount() const { return m_writeCount; }
  uint32_t readCount() const { return m_readCount; }
  uint32_t writeFailCount() const { return m_writeFail; }
  uint32_t readFailCount() const { return m_readFail; }
  const uint8_t* simFrameData() const { return m_frameBuf; }

 private:
  void regWrite(uint8_t reg, uint8_t val);

  uint8_t m_regs[256];
  uint8_t m_stuck = 0;
  uint8_t m_frameBuf[kSimFrameCap];

  Ov7670SimFault m_fault = Ov7670SimFault::NONE;
  bool m_frameAvail = true;
  uint32_t m_frameEveryN = 0;
  uint32_t m_reportedLen = kSimFrameBytes;
  uint32_t m_frameTick = 0;
  uint32_t m_writeCount = 0;
  uint32_t m_readCount = 0;
  uint32_t m_writeFail = 0;
  uint32_t m_readFail = 0;
};

}  // namespace Components

#endif
