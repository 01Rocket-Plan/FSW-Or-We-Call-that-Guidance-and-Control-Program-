#include "Components/EnvSensing/ImuDriver/ImuDriverSim.hpp"

namespace Components {

static RocketTypes::DrvState toFpp(ImuDrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

ImuDriverSim::ImuDriverSim(const char* compName)
    : ImuDriverSimComponentBase(compName), m_core(nullptr), m_tlmDiv(0) {
  m_core = imu_core_create(m_dev.busOps());
}

ImuDriverSim::~ImuDriverSim() { imu_core_destroy(m_core); }

void ImuDriverSim::loadParams() {
  if (!m_core) return;
  Fw::ParamValid v;
  ImuParams p = imu_params_default();
  p.stuckN = this->paramGet_stuckN(v);
  p.busErrThreshold = this->paramGet_busErrThreshold(v);
  p.whoamiFailThreshold = this->paramGet_whoamiFailThreshold(v);
  p.selftestSamples = this->paramGet_selftestSamples(v);
  p.gyroSatThresh_rads = this->paramGet_gyroSatThresh_rads(v);
  p.accelSatThresh_mps2 = this->paramGet_accelSatThresh_mps2(v);
  imu_core_set_params(m_core, &p);
}

void ImuDriverSim::parameterUpdated(FwPrmIdType) { loadParams(); }
void ImuDriverSim::parametersLoaded() { loadParams(); }

void ImuDriverSim::injectFault_handler(FwIndexType, U8 code) {
  m_dev.setInject(code);
  this->log_ACTIVITY_LO_SimInject(code);
  this->tlmWrite_injectCode(code);
}

void ImuDriverSim::emitSample(const ImuSampleOut& s) {
  if (!this->isConnected_imuOut_OutputPort(0)) return;
  RocketTypes::SampleHeader hdr(s.t_s, s.seq, s.valid);
  RocketTypes::ImuSample::Type_of_acc_mps2 acc = {s.acc_mps2[0], s.acc_mps2[1], s.acc_mps2[2]};
  RocketTypes::ImuSample::Type_of_gyro_rads gyro = {s.gyro_rads[0], s.gyro_rads[1], s.gyro_rads[2]};
  this->imuOut_out(0, RocketTypes::ImuSample(hdr, acc, gyro, s.gyroSat));
}

void ImuDriverSim::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  ImuSampleOut out;
  imu_core_tick(m_core, t_s, &out);
  const ImuDrvState st = imu_core_state(m_core);
  if (st == IMU_DRV_READY) {
    /* opportunistic ready log throttled via tlm path */
  }
  if (st == IMU_DRV_FAULT) {
    this->log_WARNING_HI_EnterFault(static_cast<U8>(imu_core_active(m_core)),
                                    static_cast<U8>(imu_core_fault(m_core)));
  }
  emitSample(out);
  if (++m_tlmDiv >= 1000) {
    m_tlmDiv = 0;
    this->tlmWrite_drvState(toFpp(st));
    this->tlmWrite_activeSensor(static_cast<U8>(imu_core_active(m_core)));
    this->tlmWrite_injectCode(m_dev.inject());
  }
}

}  // namespace Components
