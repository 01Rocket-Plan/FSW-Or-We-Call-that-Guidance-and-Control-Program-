#include "Components/EnvSensing/UsMpu6050/UsMpu6050.hpp"
#include <cstring>

namespace Components {

static RocketTypes::DrvState toFpp(ImuDrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

UsMpu6050::UsMpu6050(const char* compName)
    : UsMpu6050ComponentBase(compName), m_core(nullptr), m_tlmDiv(0), m_lastState(IMU_DRV_OFF) {
  ImuBusOps ops;
  ops.ctx = this;
  ops.writeReg = &UsMpu6050::busWrite;
  ops.readReg = &UsMpu6050::busRead;
  m_core = mpu_core_create(ops);
}

UsMpu6050::~UsMpu6050() { mpu_core_destroy(m_core); }

void UsMpu6050::loadParams() {
  if (!m_core) return;
  Fw::ParamValid v;
  MpuParams p = mpu_params_default();
  p.stuckN = this->paramGet_stuckN(v);
  p.busErrThreshold = this->paramGet_busErrThreshold(v);
  p.whoamiFailThreshold = this->paramGet_whoamiFailThreshold(v);
  p.selftestSamples = this->paramGet_selftestSamples(v);
  p.gyroSatThresh_rads = this->paramGet_gyroSatThresh_rads(v);
  p.accelSatThresh_mps2 = this->paramGet_accelSatThresh_mps2(v);
  p.i2cAddr = this->paramGet_i2cAddr(v);
  mpu_core_set_params(m_core, &p);
}

void UsMpu6050::parameterUpdated(FwPrmIdType) { loadParams(); }
void UsMpu6050::parametersLoaded() { loadParams(); }

bool UsMpu6050::busWrite(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
  auto* self = static_cast<UsMpu6050*>(ctx);
  U8 buf[16];
  if (len > sizeof(buf)) return false;
  std::memcpy(buf, data, len);
  Fw::Buffer wb(buf, len);
  return self->busWrite_out(0, addr, reg, wb) == EnvSensingBus::BusStatus::OK;
}

bool UsMpu6050::busRead(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
  auto* self = static_cast<UsMpu6050*>(ctx);
  Fw::Buffer rb(data, len);
  return self->busRead_out(0, addr, reg, rb) == EnvSensingBus::BusStatus::OK;
}

void UsMpu6050::emitSample(const MpuSampleOut& s) {
  if (!this->isConnected_mpuOut_OutputPort(0)) return;
  RocketTypes::SampleHeader hdr(s.t_s, s.seq, s.valid);
  RocketTypes::Mpu6050Sample::Type_of_accel_sensor_mps2 acc = {s.accel_sensor_mps2[0], s.accel_sensor_mps2[1], s.accel_sensor_mps2[2]};
  RocketTypes::Mpu6050Sample::Type_of_gyro_sensor_rads gyro = {s.gyro_sensor_rads[0], s.gyro_sensor_rads[1], s.gyro_sensor_rads[2]};
  this->mpuOut_out(0, RocketTypes::Mpu6050Sample(hdr, acc, gyro, s.accelSat, s.gyroSat));
}

void UsMpu6050::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  MpuSampleOut out;
  mpu_core_tick(m_core, t_s, &out);
  const ImuDrvState st = mpu_core_state(m_core);
  if (st != m_lastState) {
    if (st == IMU_DRV_READY) this->log_ACTIVITY_HI_EnterReady();
    if (st == IMU_DRV_FAULT) this->log_WARNING_HI_EnterFault(static_cast<U8>(mpu_core_fault(m_core)));
    m_lastState = st;
  }
  if (out.gyroSat) this->log_WARNING_LO_GyroSaturated();
  if (out.accelSat) this->log_WARNING_LO_AccelSaturated();
  emitSample(out);
  if (++m_tlmDiv >= 1000) {
    m_tlmDiv = 0;
    this->tlmWrite_drvState(toFpp(st));
    this->tlmWrite_busErrCount(mpu_core_bus_err(m_core));
    this->tlmWrite_stuckCount(mpu_core_stuck(m_core));
  }
}

}  // namespace Components
