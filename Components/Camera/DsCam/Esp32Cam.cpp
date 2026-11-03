#include "Components/Camera/DsCam/Esp32Cam.hpp"

namespace Components {

namespace {
CamStatus toFppStatus(const CamStatus& s) { return s; }
}  // namespace

uint32_t Esp32CamI2cLink::send(const uint8_t* buf, uint32_t len) {
  if (buf == nullptr || len == 0 || m_write == nullptr) {
    return 0;
  }
  // 有界：單次最多 64 B
  U8 tmp[64];
  const U32 n = (len > sizeof(tmp)) ? static_cast<U32>(sizeof(tmp)) : len;
  for (U32 i = 0; i < n; ++i) {
    tmp[i] = buf[i];
  }
  if (!m_write(m_ctx, m_addr, tmp, n)) {
    return 0;
  }
  return n;
}

uint32_t Esp32CamI2cLink::recv(uint8_t* buf, uint32_t cap) {
  if (buf == nullptr || cap == 0) {
    return 0;
  }
  uint32_t n = 0;
  while (n < cap && m_rxHead != m_rxTail) {
    buf[n++] = m_rx[m_rxHead];
    m_rxHead = (m_rxHead + 1) % sizeof(m_rx);
  }
  return n;
}

void Esp32CamI2cLink::pushRx(const uint8_t* buf, uint32_t len) {
  if (buf == nullptr) {
    return;
  }
  for (uint32_t i = 0; i < len; ++i) {
    const uint32_t next = (m_rxTail + 1) % sizeof(m_rx);
    if (next == m_rxHead) {
      return;
    }
    m_rx[m_rxTail] = buf[i];
    m_rxTail = next;
  }
}

bool Esp32Cam::i2cWriteThunk(void* ctx, U32 addr, U8* data, U32 len) {
  auto* self = static_cast<Esp32Cam*>(ctx);
  Fw::Buffer buf(data, len);
  return self->i2cWrite_out(0, addr, buf) == Drv::I2cStatus::I2C_OK;
}

bool Esp32Cam::i2cWriteReadThunk(void* ctx, U32 addr, U8* wr, U32 wrLen, U8* rd, U32 rdLen) {
  auto* self = static_cast<Esp32Cam*>(ctx);
  Fw::Buffer wbuf(wr, wrLen);
  Fw::Buffer rbuf(rd, rdLen);
  return self->i2cWriteRead_out(0, addr, wbuf, rbuf) == Drv::I2cStatus::I2C_OK;
}

Esp32Cam::Esp32Cam(const char* compName) : Esp32CamComponentBase(compName) {
  m_link.bind(this, m_i2cAddr, &Esp32Cam::i2cWriteThunk, &Esp32Cam::i2cWriteReadThunk);
  m_core.setLink(&m_link);
  m_core.setSink(this);
  m_core.configure(CamModel::ESP32_CAM, CamRole::DS_CAM, 3, 20);
}

void Esp32Cam::loadParams() {
  Fw::ParamValid v;
  const CamModel model = this->paramGet_MODEL(v);
  const CamRole role = this->paramGet_ROLE(v);
  const U8 miss = this->paramGet_MISS_LIMIT(v);
  const U32 frameLim = this->paramGet_FRAME_LIMIT(v);
  const U32 clk = this->paramGet_I2C_CLOCK_HZ(v);
  m_i2cAddr = this->paramGet_I2C_ADDR(v);
  m_pinSda = this->paramGet_I2C_PIN_SDA(v);
  m_pinScl = this->paramGet_I2C_PIN_SCL(v);
  (void)m_pinSda;
  (void)m_pinScl;  // placeholder until Anatoly pin table; physical layer uses addr
  m_core.configure(model, role, miss, frameLim);
  m_core.setI2cClockHz(clk);
  m_link.setAddr(m_i2cAddr);
}

void Esp32Cam::parameterUpdated(FwPrmIdType) { loadParams(); }
void Esp32Cam::parametersLoaded() { loadParams(); }

void Esp32Cam::writeTlm() {
  this->tlmWrite_Status(toFppStatus(m_core.status()));
}

void Esp32Cam::respond(FwOpcodeType opCode, U32 cmdSeq, Esp32CamCmdResult r) {
  const Fw::CmdResponse resp =
      (r == Esp32CamCmdResult::OK) ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR;
  this->cmdResponse_out(opCode, cmdSeq, resp);
}

void Esp32Cam::sched1Hz_handler(FwIndexType, U32) {
  m_core.tick1Hz();
  writeTlm();
}

void Esp32Cam::sched100Hz_handler(FwIndexType, U32) {
  m_core.tick100Hz();
}

void Esp32Cam::POWER_ON_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.powerOn()));
}

void Esp32Cam::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.reset()));
}

void Esp32Cam::CAPTURE_START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.captureStart()));
}

void Esp32Cam::CAPTURE_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  respond(opCode, cmdSeq, esp32CamCmdResult(m_core.captureStop()));
}

void Esp32Cam::onStateChange(CamState from, CamState to) {
  this->log_ACTIVITY_HI_StateChanged(from, to);
  writeTlm();
}

void Esp32Cam::onFault(CamFault code, uint32_t detail) {
  this->log_WARNING_HI_FaultRaised(code, detail);
  writeTlm();
}

}  // namespace Components
