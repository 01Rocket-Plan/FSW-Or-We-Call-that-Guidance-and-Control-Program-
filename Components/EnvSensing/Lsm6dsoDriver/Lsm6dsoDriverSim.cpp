#include "Components/EnvSensing/Lsm6dsoDriver/Lsm6dsoDriverSim.hpp"

namespace Components {

static RocketTypes::DrvState toFpp(LsmDrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

Lsm6dsoDriverSim::Lsm6dsoDriverSim(const char* compName)
    : Lsm6dsoDriverSimComponentBase(compName), m_core(nullptr), m_tlmDiv(0), m_lastState(LSM_DRV_OFF) {
  m_core = lsm_core_create(m_dev.busOps());
}

Lsm6dsoDriverSim::~Lsm6dsoDriverSim() { lsm_core_destroy(m_core); }

void Lsm6dsoDriverSim::loadParams() {
  if (!m_core) return;
  Fw::ParamValid v;
  LsmParams p = lsm_params_default();
  p.stuckN = this->paramGet_stuckN(v);
  p.busErrThreshold = this->paramGet_busErrThreshold(v);
  p.whoamiFailThreshold = this->paramGet_whoamiFailThreshold(v);
  p.selftestSamples = this->paramGet_selftestSamples(v);
  p.gyroSatThresh_rads = this->paramGet_gyroSatThresh_rads(v);
  p.accelSatThresh_mps2 = this->paramGet_accelSatThresh_mps2(v);
  lsm_core_set_params(m_core, &p);
}

void Lsm6dsoDriverSim::parameterUpdated(FwPrmIdType) { loadParams(); }
void Lsm6dsoDriverSim::parametersLoaded() { loadParams(); }

void Lsm6dsoDriverSim::injectFault_handler(FwIndexType, U8 code) {
  m_dev.setInject(code);
  this->log_ACTIVITY_LO_SimInject(code);
  this->tlmWrite_injectCode(code);
}

void Lsm6dsoDriverSim::emitSample(const LsmSampleOut& s) {
  if (!this->isConnected_imuOut_OutputPort(0)) return;
  RocketTypes::SampleHeader hdr(s.t_s, s.seq, s.valid);
  RocketTypes::ImuSample::Type_of_acc_mps2 acc = {s.acc_mps2[0], s.acc_mps2[1], s.acc_mps2[2]};
  RocketTypes::ImuSample::Type_of_gyro_rads gyro = {s.gyro_rads[0], s.gyro_rads[1], s.gyro_rads[2]};
  this->imuOut_out(0, RocketTypes::ImuSample(hdr, acc, gyro, s.gyroSat));
}

void Lsm6dsoDriverSim::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  LsmSampleOut out;
  lsm_core_tick(m_core, t_s, &out);
  const LsmDrvState st = lsm_core_state(m_core);
  if (st != m_lastState) {  // 只在狀態剛改變時記（原本 FAULT 每拍記一次 → 1 kHz 洪水）
    if (st == LSM_DRV_READY) this->log_ACTIVITY_HI_EnterReady();
    if (st == LSM_DRV_FAULT) this->log_WARNING_HI_EnterFault(static_cast<U8>(lsm_core_fault(m_core)));
    m_lastState = st;
    this->tlmWrite_drvState(toFpp(st));
  }
  emitSample(out);
  if (++m_tlmDiv >= 1000) {
    m_tlmDiv = 0;
    this->tlmWrite_drvState(toFpp(st));
    this->tlmWrite_injectCode(m_dev.inject());
  }
}

}  // namespace Components
