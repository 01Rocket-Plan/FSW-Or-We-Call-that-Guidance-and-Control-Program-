#include "Components/EnvSensing/ImuDriver/ImuDriver.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include <cstring>

namespace Components {

static RocketTypes::DrvState toFpp(ImuDrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

ImuDriver::ImuDriver(const char* compName)
    : ImuDriverComponentBase(compName), m_core(nullptr), m_tlmDiv(0),
      m_lastState(IMU_DRV_OFF), m_lastActive(IMU_SENS_PRIMARY) {
  ImuBusOps ops;
  ops.ctx = this;
  ops.writeReg = &ImuDriver::busWrite;
  ops.readReg = &ImuDriver::busRead;
  m_core = imu_core_create(ops);
}

ImuDriver::~ImuDriver() { imu_core_destroy(m_core); }

void ImuDriver::loadParams() {
  if (!m_core) return;
  Fw::ParamValid v;
  ImuParams p = imu_params_default();
  p.stuckN = this->paramGet_stuckN(v);
  p.busErrThreshold = this->paramGet_busErrThreshold(v);
  p.whoamiFailThreshold = this->paramGet_whoamiFailThreshold(v);
  p.selftestSamples = this->paramGet_selftestSamples(v);
  p.gyroSatThresh_rads = this->paramGet_gyroSatThresh_rads(v);
  p.accelSatThresh_mps2 = this->paramGet_accelSatThresh_mps2(v);
  p.primaryAddr = this->paramGet_primaryI2cAddr(v);
  p.backupAddr = this->paramGet_backupI2cAddr(v);
  p.dcmPrimary[0] = this->paramGet_dcmP00(v);
  p.dcmPrimary[1] = this->paramGet_dcmP01(v);
  p.dcmPrimary[2] = this->paramGet_dcmP02(v);
  p.dcmPrimary[3] = this->paramGet_dcmP10(v);
  p.dcmPrimary[4] = this->paramGet_dcmP11(v);
  p.dcmPrimary[5] = this->paramGet_dcmP12(v);
  p.dcmPrimary[6] = this->paramGet_dcmP20(v);
  p.dcmPrimary[7] = this->paramGet_dcmP21(v);
  p.dcmPrimary[8] = this->paramGet_dcmP22(v);
  p.dcmBackup[0] = this->paramGet_dcmB00(v);
  p.dcmBackup[1] = this->paramGet_dcmB01(v);
  p.dcmBackup[2] = this->paramGet_dcmB02(v);
  p.dcmBackup[3] = this->paramGet_dcmB10(v);
  p.dcmBackup[4] = this->paramGet_dcmB11(v);
  p.dcmBackup[5] = this->paramGet_dcmB12(v);
  p.dcmBackup[6] = this->paramGet_dcmB20(v);
  p.dcmBackup[7] = this->paramGet_dcmB21(v);
  p.dcmBackup[8] = this->paramGet_dcmB22(v);
  imu_core_set_params(m_core, &p);
}

void ImuDriver::parameterUpdated(FwPrmIdType) { loadParams(); }
void ImuDriver::parametersLoaded() { loadParams(); }

bool ImuDriver::busWrite(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
  auto* self = static_cast<ImuDriver*>(ctx);
  U8 buf[16];
  if (len > sizeof(buf)) return false;
  std::memcpy(buf, data, len);
  Fw::Buffer wb(buf, len);
  const bool primary = (imu_core_active(self->m_core) == IMU_SENS_PRIMARY);
  EnvSensingBus::BusStatus st = primary
      ? self->primaryWrite_out(0, addr, reg, wb)
      : self->backupWrite_out(0, addr, reg, wb);
  return st == EnvSensingBus::BusStatus::OK;
}

bool ImuDriver::busRead(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
  auto* self = static_cast<ImuDriver*>(ctx);
  Fw::Buffer rb(data, len);
  const bool primary = (imu_core_active(self->m_core) == IMU_SENS_PRIMARY);
  EnvSensingBus::BusStatus st = primary
      ? self->primaryRead_out(0, addr, reg, rb)
      : self->backupRead_out(0, addr, reg, rb);
  return st == EnvSensingBus::BusStatus::OK;
}

void ImuDriver::emitSample(const ImuSampleOut& s) {
  if (!this->isConnected_imuOut_OutputPort(0)) return;
  RocketTypes::SampleHeader hdr(s.t_s, s.seq, s.valid);
  RocketTypes::ImuSample::Type_of_acc_mps2 acc = {s.acc_mps2[0], s.acc_mps2[1], s.acc_mps2[2]};
  RocketTypes::ImuSample::Type_of_gyro_rads gyro = {s.gyro_rads[0], s.gyro_rads[1], s.gyro_rads[2]};
  this->imuOut_out(0, RocketTypes::ImuSample(hdr, acc, gyro, s.gyroSat));
}

void ImuDriver::writeTlm() {
  this->tlmWrite_drvState(toFpp(imu_core_state(m_core)));
  this->tlmWrite_activeSensor(static_cast<U8>(imu_core_active(m_core)));
  this->tlmWrite_busErrCount(imu_core_bus_err(m_core));
  this->tlmWrite_stuckCount(imu_core_stuck(m_core));
  this->tlmWrite_failoverCount(imu_core_failover(m_core));
}

void ImuDriver::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  ImuSampleOut out;
  imu_core_tick(m_core, t_s, &out);

  const ImuDrvState st = imu_core_state(m_core);
  const ImuSensorId act = imu_core_active(m_core);
  if (st != m_lastState) {
    if (st == IMU_DRV_READY) this->log_ACTIVITY_HI_EnterReady(static_cast<U8>(act));
    if (st == IMU_DRV_FAULT) this->log_WARNING_HI_EnterFault(static_cast<U8>(act), static_cast<U8>(imu_core_fault(m_core)));
    m_lastState = st;
  }
  if (act != m_lastActive) {
    this->log_WARNING_HI_Failover(static_cast<U8>(m_lastActive), static_cast<U8>(act));
    m_lastActive = act;
  }
  if (out.gyroSat) this->log_WARNING_LO_GyroSaturated(static_cast<U8>(act));
  if (out.accelSat) this->log_WARNING_LO_AccelSaturated(static_cast<U8>(act));

  emitSample(out);
  if (++m_tlmDiv >= 1000) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

}  // namespace Components
