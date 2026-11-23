#include "Components/EnvSensing/Lsm6dsoDriver/Lsm6dsoDriver.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include <cstring>

namespace Components {

static RocketTypes::DrvState toFpp(LsmDrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

Lsm6dsoDriver::Lsm6dsoDriver(const char* compName)
    : Lsm6dsoDriverComponentBase(compName),
      m_core(nullptr),
      m_tlmDiv(0),
      m_lastState(LSM_DRV_OFF),
      m_gyroSatLatched(false),
      m_accelSatLatched(false) {
  LsmBusOps ops;
  ops.ctx = this;
  ops.writeReg = &Lsm6dsoDriver::busWrite;
  ops.readReg = &Lsm6dsoDriver::busRead;
  m_core = lsm_core_create(ops);
}

Lsm6dsoDriver::~Lsm6dsoDriver() { lsm_core_destroy(m_core); }

void Lsm6dsoDriver::loadParams() {
  if (!m_core) return;
  Fw::ParamValid v;
  LsmParams p = lsm_params_default();
  p.stuckN = this->paramGet_stuckN(v);
  p.busErrThreshold = this->paramGet_busErrThreshold(v);
  p.whoamiFailThreshold = this->paramGet_whoamiFailThreshold(v);
  p.selftestSamples = this->paramGet_selftestSamples(v);
  p.gyroSatThresh_rads = this->paramGet_gyroSatThresh_rads(v);
  p.accelSatThresh_mps2 = this->paramGet_accelSatThresh_mps2(v);
  p.i2cAddr = this->paramGet_i2cAddr(v);
  p.dcm[0] = this->paramGet_dcm00(v);
  p.dcm[1] = this->paramGet_dcm01(v);
  p.dcm[2] = this->paramGet_dcm02(v);
  p.dcm[3] = this->paramGet_dcm10(v);
  p.dcm[4] = this->paramGet_dcm11(v);
  p.dcm[5] = this->paramGet_dcm12(v);
  p.dcm[6] = this->paramGet_dcm20(v);
  p.dcm[7] = this->paramGet_dcm21(v);
  p.dcm[8] = this->paramGet_dcm22(v);
  lsm_core_set_params(m_core, &p);
}

void Lsm6dsoDriver::parameterUpdated(FwPrmIdType) { loadParams(); }
void Lsm6dsoDriver::parametersLoaded() { loadParams(); }

bool Lsm6dsoDriver::busWrite(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
  auto* self = static_cast<Lsm6dsoDriver*>(ctx);
  if (!self->isConnected_busWrite_OutputPort(0)) return false;
  U8 buf[16];
  if (len > sizeof(buf)) return false;
  std::memcpy(buf, data, len);
  Fw::Buffer wb(buf, len);
  return self->busWrite_out(0, addr, reg, wb) == EnvSensingBus::BusStatus::OK;
}

bool Lsm6dsoDriver::busRead(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
  auto* self = static_cast<Lsm6dsoDriver*>(ctx);
  if (!self->isConnected_busRead_OutputPort(0)) return false;
  Fw::Buffer rb(data, len);
  return self->busRead_out(0, addr, reg, rb) == EnvSensingBus::BusStatus::OK;
}

void Lsm6dsoDriver::emitSample(const LsmSampleOut& s) {
  if (!this->isConnected_imuOut_OutputPort(0)) return;
  RocketTypes::SampleHeader hdr(s.t_s, s.seq, s.valid);
  RocketTypes::ImuSample::Type_of_acc_mps2 acc = {s.acc_mps2[0], s.acc_mps2[1], s.acc_mps2[2]};
  RocketTypes::ImuSample::Type_of_gyro_rads gyro = {s.gyro_rads[0], s.gyro_rads[1], s.gyro_rads[2]};
  this->imuOut_out(0, RocketTypes::ImuSample(hdr, acc, gyro, s.gyroSat));
}

void Lsm6dsoDriver::writeTlm() {
  this->tlmWrite_drvState(toFpp(lsm_core_state(m_core)));
  this->tlmWrite_busErrCount(lsm_core_bus_err(m_core));
  this->tlmWrite_stuckCount(lsm_core_stuck(m_core));
  this->tlmWrite_faultReason(static_cast<U8>(lsm_core_fault(m_core)));
}

void Lsm6dsoDriver::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  LsmSampleOut out;
  lsm_core_tick(m_core, t_s, &out);

  const LsmDrvState st = lsm_core_state(m_core);
  if (st != m_lastState) {
    if (st == LSM_DRV_READY) this->log_ACTIVITY_HI_EnterReady();
    if (st == LSM_DRV_FAULT) this->log_WARNING_HI_EnterFault(static_cast<U8>(lsm_core_fault(m_core)));
    m_lastState = st;
    writeTlm();  // 狀態剛改變那一拍立刻寫，之後 1 Hz
    m_tlmDiv = 0;
  }
  // 飽和事件只在「剛開始飽和」那一拍記（1 kHz 每拍記會洪水，同 #38）
  if (out.gyroSat && !m_gyroSatLatched) this->log_WARNING_LO_GyroSaturated();
  if (out.accelSat && !m_accelSatLatched) this->log_WARNING_LO_AccelSaturated();
  if (out.valid) {
    m_gyroSatLatched = out.gyroSat;
    m_accelSatLatched = out.accelSat;
  }

  emitSample(out);
  if (++m_tlmDiv >= 1000) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

}  // namespace Components
