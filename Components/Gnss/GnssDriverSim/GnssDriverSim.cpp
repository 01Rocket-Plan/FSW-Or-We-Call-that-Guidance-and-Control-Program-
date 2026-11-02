#include "Components/Gnss/GnssDriverSim/GnssDriverSim.hpp"
#include "Components/Gnss/GnssDriver/GnssToFpp.hpp"

namespace Components {

static const F64 kDeg2Rad = 3.14159265358979323846 / 180.0;

static RocketTypes::DrvState toFpp(Gnss::DrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

Components::GnssLockStage GnssDriverSim::toFppLock(Gnss::LockStage s) {
  return Components::GnssLockStage(static_cast<Components::GnssLockStage::T>(static_cast<U8>(s)));
}

GnssDriverSim::GnssDriverSim(const char* compName)
    : GnssDriverSimComponentBase(compName), m_core(m_dev, *this), m_lastT_s(0.0), m_haveLastT(false),
      m_faultCount(0), m_started(false), m_lastDataAge(0.f) {}

Gnss::GnssCoreParams GnssDriverSim::readParams() {
  Fw::ParamValid pv; Gnss::GnssCoreParams p;
  p.hdopMax = this->paramGet_HDOP_MAX(pv);
  // ticks = ms / rgPeriod_ms（rg100Hz → 10 ms）；見 GnssCore.hpp
  p.rgPeriod_ms = 10;
  p.pollTracking_ms = this->paramGet_POLL_TRACKING_MS(pv);
  p.pollReacqFast_ms = this->paramGet_POLL_REACQ_FAST_MS(pv);
  p.pollReacqMed_ms = this->paramGet_POLL_REACQ_MED_MS(pv);
  p.pollReacqSlow_ms = this->paramGet_POLL_REACQ_SLOW_MS(pv);
  p.reacqFastDur_s = this->paramGet_REACQ_FAST_S(pv);
  p.reacqMedDur_s = this->paramGet_REACQ_MED_S(pv);
  p.reacqSlowDur_s = this->paramGet_REACQ_SLOW_S(pv);
  p.initAcqTimeout_s = this->paramGet_INIT_ACQ_TIMEOUT_S(pv);
  p.selftestTimeout_s = this->paramGet_SELFTEST_TIMEOUT_S(pv);
  p.lossNoGga_s = this->paramGet_LOSS_NO_GGA_S(pv);
  p.stuckPolls = this->paramGet_STUCK_POLLS(pv);
  return p;
}

void GnssDriverSim::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  const F64 dt_s = m_haveLastT ? (t_s - m_lastT_s) : 0.01;
  m_lastT_s = t_s; m_haveLastT = true;

  m_core.setParams(readParams());
  if (!m_started) { m_core.start(); m_started = true; }

  m_dev.advance(dt_s > 0.0 ? dt_s : 0.0);
  m_core.tick(t_s);

  this->tlmWrite_HsState(toFpp(m_core.state()));
  this->tlmWrite_LockStage(toFppLock(m_core.lockStage()));
  this->tlmWrite_FaultCount(m_faultCount);
  this->tlmWrite_DataAge(m_lastDataAge);
}

void GnssDriverSim::SIM_SET_TRAJ_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F64 lat0_deg, F64 lon0_deg, F32 alt0_m,
                                            F32 velE_mps, F32 velN_mps, F32 velU_mps) {
  const float v[3] = {velE_mps, velN_mps, velU_mps};
  m_dev.setTrajectory(lat0_deg * kDeg2Rad, lon0_deg * kDeg2Rad, alt0_m, v);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GnssDriverSim::SIM_SET_NOISE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 h, F32 v, F32 vel) {
  if (h < 0.f || v < 0.f || vel < 0.f) { this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR); return; }
  m_dev.setNoise(h, v, vel);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GnssDriverSim::SIM_SET_LATENCY_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 latency_s, F32 dropout_prob) {
  if (latency_s < 0.f || dropout_prob < 0.f || dropout_prob > 1.f) {
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR); return;
  }
  m_dev.setLatency(latency_s); m_dev.setDropoutProb(dropout_prob);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GnssDriverSim::SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Components::GnssSimFault& fault) {
  m_dev.setFault(static_cast<Gnss::GnssSimFault>(fault.e));
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GnssDriverSim::SIM_SET_HDOP_BIAS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 bias) {
  m_dev.setHdopBias(bias);
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GnssDriverSim::onFix(const Gnss::GnssFix& f) {
  m_lastDataAge = f.dataAge_s;
  this->tlmWrite_FixSeq(f.seq);
  if (!this->isConnected_fixOut_OutputPort(0)) return;
  this->fixOut_out(0, Gnss::toFpp(f));
}

void GnssDriverSim::onStateChange(Gnss::DrvState from, Gnss::DrvState to) {
  this->log_ACTIVITY_HI_StateChanged(toFpp(from), toFpp(to));
}

void GnssDriverSim::onLockStageChange(Gnss::LockStage from, Gnss::LockStage to) {
  this->log_ACTIVITY_HI_LockStageChanged(toFppLock(from), toFppLock(to));
}

void GnssDriverSim::onFault(Gnss::GnssFault fault, uint32_t count) {
  m_faultCount++;
  this->log_WARNING_HI_Fault(static_cast<U8>(fault), count);
}

void GnssDriverSim::onFixFailedOpenSky() {
  this->log_WARNING_HI_FixFailedOpenSky();
}

}  // namespace Components
