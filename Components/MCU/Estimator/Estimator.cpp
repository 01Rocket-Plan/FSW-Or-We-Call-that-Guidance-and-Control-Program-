// ======================================================================
// \title  Estimator.cpp
// \author MCU 組（EdwardWang+Claude）
// \brief  F´ 外殼：搬樣本進 core、1 kHz step、扇出 StateOut、10 Hz 遙測。
// ======================================================================
#include "Components/MCU/Estimator/Estimator.hpp"

namespace Mcu {

namespace {
constexpr U32 kTlmDivider = 100;  // 1 kHz → 10 Hz
}

Estimator::Estimator(const char* compName)
    : EstimatorComponentBase(compName),
      m_tlmDiv(0),
      m_lastAligned(false),
      m_lastFresh(false),
      m_lastOnPad(true) {}

Estimator::~Estimator() {}

F64 Estimator::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void Estimator::loadParams() {
  Fw::ParamValid v;
  EstimatorParams p;
  p.g_mps2 = this->paramGet_g_mps2(v);
  p.padAlign_s = this->paramGet_padAlign_s(v);
  p.accGate_mps2 = this->paramGet_accGate_mps2(v);
  p.rateGate_rads = this->paramGet_rateGate_rads(v);
  p.kLevel_rads = this->paramGet_kLevel_rads(v);
  p.gyroBiasTau_s = this->paramGet_gyroBiasTau_s(v);
  p.qAcc_m2ps4 = this->paramGet_qAcc_m2ps4(v);
  p.rBaro_m2 = this->paramGet_rBaro_m2(v);
  p.baroGate_m = this->paramGet_baroGate_m(v);
  p.rZupt_m2ps2 = this->paramGet_rZupt_m2ps2(v);
  p.altRefTau_s = this->paramGet_altRefTau_s(v);
  p.imuStale_s = this->paramGet_imuStale_s(v);
  p.baroMaxAge_s = this->paramGet_baroMaxAge_s(v);
  p.dtMax_s = this->paramGet_dtMax_s(v);
  m_core.setParams(p);
}

void Estimator::parameterUpdated(FwPrmIdType) { loadParams(); }
void Estimator::parametersLoaded() { loadParams(); }

void Estimator::imuIn_handler(FwIndexType, const RocketTypes::ImuSample& s) {
  ImuIn in;
  in.t_s = s.get_hdr().get_t_s();
  in.seq = s.get_hdr().get_seq();
  in.valid = s.get_hdr().get_valid();
  for (U32 i = 0; i < 3; ++i) {
    in.acc_mps2[i] = s.get_acc_mps2()[i];
    in.gyro_rads[i] = s.get_gyro_rads()[i];
  }
  in.gyroSat = s.get_gyroSat();
  m_core.pushImu(in);
}

void Estimator::baroIn_handler(FwIndexType, const RocketTypes::BaroSample& s) {
  BaroIn in;
  in.t_s = s.get_hdr().get_t_s();
  in.seq = s.get_hdr().get_seq();
  in.valid = s.get_hdr().get_valid();
  in.alt_m = s.get_alt_m();
  m_core.pushBaro(in);
}

void Estimator::modeIn_handler(FwIndexType, const RocketTypes::FlightModeId& mode) {
  // 只有 PAD_IDLE／ARMED 算墊上。其餘（含收不到模式以外的任何值）一律當在飛：寧可不修水平。
  const bool onPad = (mode == RocketTypes::FlightModeId::PAD_IDLE) ||
                     (mode == RocketTypes::FlightModeId::ARMED);
  m_core.setOnPad(onPad);
}

void Estimator::schedIn_handler(FwIndexType, U32) {
  StateOutC o;
  m_core.step(this->nowSeconds(), o);
  const EstimatorDiag& d = m_core.diag();

  if (d.aligned && !m_lastAligned) this->log_ACTIVITY_HI_Aligned(o.tilt_rad);
  m_lastAligned = d.aligned;
  if (d.aligned) {
    if (m_lastFresh && !o.imuValid) this->log_WARNING_HI_ImuStale();
    if (!m_lastFresh && o.imuValid) this->log_ACTIVITY_HI_ImuRecovered();
    m_lastFresh = o.imuValid;
  }
  if (m_lastOnPad && !d.onPad) this->log_ACTIVITY_HI_LeftPad();
  m_lastOnPad = d.onPad;

  const RocketTypes::StateOut::Type_of_q_wxyz q = {o.q_wxyz[0], o.q_wxyz[1], o.q_wxyz[2], o.q_wxyz[3]};
  const RocketTypes::StateOut::Type_of_rate_body_rads w = {o.rate_body_rads[0], o.rate_body_rads[1],
                                                           o.rate_body_rads[2]};
  const RocketTypes::StateOut::Type_of_vel_enu_mps v = {o.vel_enu_mps[0], o.vel_enu_mps[1], o.vel_enu_mps[2]};
  const RocketTypes::StateOut s(o.seq, o.t_s, q, w, o.tilt_rad, v, o.velValid, o.alt_m, o.baroAge_s,
                                o.accAxial_mps2, o.imuValid, o.gyroSat);
  for (FwIndexType i = 0; i < this->getNum_stateOut_OutputPorts(); ++i) {
    if (this->isConnected_stateOut_OutputPort(i)) this->stateOut_out(i, s);
  }

  if (++m_tlmDiv >= kTlmDivider) {
    m_tlmDiv = 0;
    this->writeTlm(o);
  }
}

void Estimator::writeTlm(const StateOutC& o) {
  const EstimatorDiag& d = m_core.diag();
  this->tlmWrite_aligned(d.aligned);
  this->tlmWrite_onPad(d.onPad);
  this->tlmWrite_tilt_rad(o.tilt_rad);
  this->tlmWrite_alt_m(o.alt_m);
  this->tlmWrite_velU_mps(o.vel_enu_mps[2]);
  this->tlmWrite_accAxial_mps2(o.accAxial_mps2);
  this->tlmWrite_baroAge_s(o.baroAge_s);
  this->tlmWrite_gyroBiasX_rads(d.gyroBias_rads[0]);
  this->tlmWrite_gyroBiasY_rads(d.gyroBias_rads[1]);
  this->tlmWrite_gyroBiasZ_rads(d.gyroBias_rads[2]);
  this->tlmWrite_baroRejectCount(d.baroRejectCount);
  this->tlmWrite_imuStaleCount(d.imuStaleCount);
}

}  // namespace Mcu
