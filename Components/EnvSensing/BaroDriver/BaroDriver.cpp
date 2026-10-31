#include "Components/EnvSensing/BaroDriver/BaroDriver.hpp"
#include "Components/EnvSensing/BaroDriver/Bme680Regs.hpp"

namespace Components {

static RocketTypes::DrvState toFpp(DrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

BaroDriver::BaroDriver(const char* compName)
    : BaroDriverComponentBase(compName), m_core(*this, *this), m_addr(Bme680Reg::I2C_ADDR_PRIMARY), m_tlmDiv(0) {}

void BaroDriver::configure(U32 i2cAddr) { m_addr = i2cAddr; }

void BaroDriver::loadParams() {
  Fw::ParamValid v;
  Bme680Config c = m_core.config();
  c.p0_Pa = this->paramGet_P0_PA(v);
  c.stuckN = this->paramGet_STUCK_N(v);
  c.errMax = this->paramGet_ERR_MAX(v);
  c.gasWait_ms = this->paramGet_GAS_WAIT_MS(v);
  c.heaterTemp_C = this->paramGet_HEATER_TEMP_C(v);
  m_core.setConfig(c);
}

void BaroDriver::parameterUpdated(FwPrmIdType) { loadParams(); }
void BaroDriver::parametersLoaded() { loadParams(); }

void BaroDriver::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  m_core.tick(t_s);
  if (++m_tlmDiv >= 100) {  // rg100Hz 下 1 Hz 遙測
    m_tlmDiv = 0;
    writeTlm();
  }
}

void BaroDriver::writeTlm() {
  this->tlmWrite_HsState(toFpp(m_core.state()));
  this->tlmWrite_ErrorCount(m_core.errorCount());
  this->tlmWrite_RefPress_Pa(m_core.p0_Pa());
  this->tlmWrite_BaroSeq(m_core.baroSeq());
}

bool BaroDriver::writeReg(uint8_t reg, uint8_t val) {
  U8 data[2] = {reg, val};
  Fw::Buffer buf(data, sizeof(data));
  return this->i2cWrite_out(0, m_addr, buf) == Drv::I2cStatus::I2C_OK;
}

bool BaroDriver::readRegs(uint8_t reg, uint8_t* out, uint8_t len) {
  U8 r = reg;
  Fw::Buffer wr(&r, 1);
  Fw::Buffer rd(out, len);
  return this->i2cWriteRead_out(0, m_addr, wr, rd) == Drv::I2cStatus::I2C_OK;
}

void BaroDriver::onBaro(const BaroOut& s) {
  if (!this->isConnected_baroOut_OutputPort(0)) return;
  this->baroOut_out(0, RocketTypes::BaroSample(RocketTypes::SampleHeader(s.t_s, s.seq, s.valid), s.press_Pa, s.temp_C, s.alt_m));
}

void BaroDriver::onEnv(const EnvOut& s) {
  if (!this->isConnected_envOut_OutputPort(0)) return;
  this->envOut_out(0, RocketTypes::EnvSample(RocketTypes::SampleHeader(s.t_s, s.seq, s.valid), s.humidity_pct, s.gas_ohm));
}

void BaroDriver::onStateChange(DrvState from, DrvState to) {
  this->log_ACTIVITY_HI_StateChanged(toFpp(from), toFpp(to));
  this->tlmWrite_HsState(toFpp(to));
}

void BaroDriver::onFault(BaroFault code, uint32_t detail) {
  this->log_WARNING_HI_Fault(static_cast<U8>(code), detail);
}

}  // namespace Components
