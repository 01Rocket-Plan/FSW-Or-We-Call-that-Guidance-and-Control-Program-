// Ov3640SimDevice：OV3640 SCCB／DVP 模擬（與 Core 共用同一組介面）
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_OV3640SIMDEVICE_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_OV3640SIMDEVICE_HPP

#include "Components/Camera/Ov3640Driver/Ov3640Core.hpp"
#include "Components/Camera/Ov3640Driver/Ov3640Regs.hpp"
#include <cstdint>

namespace Components {

enum class Ov3640SimFault : uint8_t {
  NONE = 0,
  NACK_ON_WRITE = 1,
  NACK_ON_READ = 2,
  BUS_DOWN = 3,
  BAD_CHIP_ID = 4,
  STUCK_READ = 5
};

class Ov3640SimDevice : public IOv3640Sccb, public IOv3640FrameSource {
 public:
  static constexpr uint32_t kSimFrameBytes = 16;
  // Host-test capacity for multi-chunk / truncation paths (not flight RAM).
  static constexpr uint32_t kSimFrameCap = 8192;

  Ov3640SimDevice();

  void advance();
  void setFault(Ov3640SimFault f) { m_fault = f; }
  void setStuckValue(uint8_t v) { m_stuck = v; }
  void setFrameAvailable(bool avail) { m_frameAvail = avail; }
  void setFrameEveryN(uint32_t n);
  // Host-test only: report length up to kSimFrameCap (multi-chunk / truncation).
  void setReportedFrameLength(uint32_t n);

  bool writeReg(uint16_t reg, uint8_t val) override;
  bool readReg(uint16_t reg, uint8_t* val) override;
  bool pollFrame(CamFrameView& out) override;

  uint32_t writeCount() const { return m_writeCount; }
  uint32_t readCount() const { return m_readCount; }
  uint32_t writeFailCount() const { return m_writeFail; }
  uint32_t readFailCount() const { return m_readFail; }
  const uint8_t* simFrameData() const { return m_frameBuf; }

 private:
  uint8_t reg8(uint16_t reg) const;
  void regWrite(uint16_t reg, uint8_t val);

  uint8_t m_regsLo[256];
  uint8_t m_pidh = Ov3640Regs::EXPECTED_PIDH;
  uint8_t m_pidl = Ov3640Regs::EXPECTED_PIDL;
  uint8_t m_stuck = 0;
  uint8_t m_frameBuf[kSimFrameCap];

  Ov3640SimFault m_fault = Ov3640SimFault::NONE;
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
