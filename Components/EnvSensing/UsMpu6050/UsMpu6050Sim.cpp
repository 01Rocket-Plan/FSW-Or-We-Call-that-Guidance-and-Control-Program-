#include "Components/EnvSensing/UsMpu6050/UsMpu6050Sim.hpp"

namespace Components {

static RocketTypes::DrvState toFpp(ImuDrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

UsMpu6050Sim::UsMpu6050Sim(const char* compName)
    : UsMpu6050SimComponentBase(compName), m_core(nullptr), m_tlmDiv(0) {
  m_core = mpu_core_create(m_dev.busOps());
}

UsMpu6050Sim::~UsMpu6050Sim() { mpu_core_destroy(m_core); }

void UsMpu6050Sim::loadParams() {
  if (!m_core) return;
  Fw::ParamValid v;
  MpuParams p = mpu_params_default();
  p.stuckN = this->paramGet_stuckN(v);
  p.busErrThreshold = this->paramGet_busErrThreshold(v);
  p.whoamiFailThreshold = this->paramGet_whoamiFailThreshold(v);
  p.selftestSamples = this->paramGet_selftestSamples(v);
  p.gyroSatThresh_rads = this->paramGet_gyroSatThresh_rads(v);
  p.accelSatThresh_mps2 = this->paramGet_accelSatThresh_mps2(v);
  mpu_core_set_params(m_core, &p);
}

void UsMpu6050Sim::parameterUpdated(FwPrmIdType) { loadParams(); }
void UsMpu6050Sim::parametersLoaded() { loadParams(); }

void UsMpu6050Sim::injectFault_handler(FwIndexType, U8 code) {
  m_dev.setInject(code);
  this->log_ACTIVITY_LO_SimInject(code);
  this->tlmWrite_injectCode(code);
}

void UsMpu6050Sim::emitSample(const MpuSampleOut& s) {
  if (!this->isConnected_mpuOut_OutputPort(0)) return;
  RocketTypes::SampleHeader hdr(s.t_s, s.seq, s.valid);
  RocketTypes::Mpu6050Sample::Type_of_accel_sensor_mps2 acc = {
      s.accel_sensor_mps2[0], s.accel_sensor_mps2[1], s.accel_sensor_mps2[2]};
  RocketTypes::Mpu6050Sample::Type_of_gyro_sensor_rads gyro = {
      s.gyro_sensor_rads[0], s.gyro_sensor_rads[1], s.gyro_sensor_rads[2]};
  this->mpuOut_out(0, RocketTypes::Mpu6050Sample(hdr, acc, gyro, s.accelSat, s.gyroSat));
}

void UsMpu6050Sim::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  MpuSampleOut out;
  mpu_core_tick(m_core, t_s, &out);
  const ImuDrvState st = mpu_core_state(m_core);
  if (st == IMU_DRV_FAULT) {
    this->log_WARNING_HI_EnterFault(static_cast<U8>(mpu_core_fault(m_core)));
  }
  emitSample(out);
  if (++m_tlmDiv >= 1000) {
    m_tlmDiv = 0;
    this->tlmWrite_drvState(toFpp(st));
    this->tlmWrite_injectCode(m_dev.inject());
  }
}

}  // namespace Components
