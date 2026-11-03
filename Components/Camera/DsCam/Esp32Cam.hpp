// Esp32Cam：ESP32-CAM 的 F´ 元件外殼（I2C → ICamLink；邏輯在 Esp32CamCore）
#ifndef COMPONENTS_CAMERA_DSCAM_ESP32CAM_HPP
#define COMPONENTS_CAMERA_DSCAM_ESP32CAM_HPP

#include "Components/Camera/DsCam/Esp32CamCmd.hpp"
#include "Components/Camera/DsCam/Esp32CamComponentAc.hpp"
#include "Components/Camera/DsCam/Esp32CamCore.hpp"
#include "Components/Camera/DsCam/ICamLink.hpp"

namespace Components {

//! I2C 位元組適配：把 Drv.I2c 埠接到 Esp32CamCore 的 ICamLink
class Esp32CamI2cLink final : public ICamLink {
 public:
  using WriteFn = bool (*)(void* ctx, U32 addr, U8* data, U32 len);
  using WriteReadFn = bool (*)(void* ctx, U32 addr, U8* wr, U32 wrLen, U8* rd, U32 rdLen);

  void bind(void* ctx, U32 addr, WriteFn wr, WriteReadFn wrd) {
    m_ctx = ctx;
    m_addr = addr;
    m_write = wr;
    m_writeRead = wrd;
  }
  void setAddr(U32 addr) { m_addr = addr; }

  uint32_t send(const uint8_t* buf, uint32_t len) override;
  uint32_t recv(uint8_t* buf, uint32_t cap) override;

  void pushRx(const uint8_t* buf, uint32_t len);

 private:
  void* m_ctx = nullptr;
  U32 m_addr = 0;
  WriteFn m_write = nullptr;
  WriteReadFn m_writeRead = nullptr;
  uint8_t m_rx[128]{};
  uint32_t m_rxHead = 0;
  uint32_t m_rxTail = 0;
};

class Esp32Cam final : public Esp32CamComponentBase, private ICamSink {
 public:
  explicit Esp32Cam(const char* compName);
  ~Esp32Cam() override = default;

 private:
  void sched1Hz_handler(FwIndexType portNum, U32 context) override;
  void sched100Hz_handler(FwIndexType portNum, U32 context) override;

  void POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void onStateChange(CamState from, CamState to) override;
  void onFault(CamFault code, uint32_t detail) override;

  void loadParams();
  void writeTlm();
  void respond(FwOpcodeType opCode, U32 cmdSeq, Esp32CamCmdResult r);

  static bool i2cWriteThunk(void* ctx, U32 addr, U8* data, U32 len);
  static bool i2cWriteReadThunk(void* ctx, U32 addr, U8* wr, U32 wrLen, U8* rd, U32 rdLen);

  Esp32CamCore m_core;
  Esp32CamI2cLink m_link;
  U32 m_i2cAddr = 0;
  U8 m_pinSda = 0xFF;
  U8 m_pinScl = 0xFF;
};

}  // namespace Components
#endif
