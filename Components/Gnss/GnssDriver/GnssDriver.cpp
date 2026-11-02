#include "Components/Gnss/GnssDriver/GnssDriver.hpp"
#include "Components/Gnss/GnssDriver/GnssToFpp.hpp"

namespace Components {

static RocketTypes::DrvState toFppState(Gnss::DrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

Components::GnssLockStage GnssDriver::toFppLock(Gnss::LockStage s) {
  return Components::GnssLockStage(static_cast<Components::GnssLockStage::T>(static_cast<U8>(s)));
}

int32_t GnssUartAdapter::readBytes(uint8_t* buf, uint32_t max) {
  if (m_busError) { m_busError = false; return -1; }
  U32 n = 0;
  while (n < max && m_size > 0) {
    buf[n++] = m_buf[m_head];
    m_head = (m_head + 1) % CAP;
    --m_size;
  }
  return static_cast<int32_t>(n);
}

void GnssUartAdapter::push(const uint8_t* data, U32 len) {
  for (U32 i = 0; i < len; ++i) {
    if (m_size >= CAP) break;
    m_buf[m_tail] = data[i];
    m_tail = (m_tail + 1) % CAP;
    ++m_size;
  }
}

GnssDriver::GnssDriver(const char* compName)
    : GnssDriverComponentBase(compName), m_core(m_uart, *this), m_started(false), m_faultCount(0), m_lastDataAge(0.f) {}

Gnss::GnssCoreParams GnssDriver::readParams() {
  Fw::ParamValid pv;
  Gnss::GnssCoreParams p;
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

void GnssDriver::uartRecv_handler(FwIndexType, Fw::Buffer& buffer, const Drv::ByteStreamStatus& status) {
  if (status == Drv::ByteStreamStatus::OP_OK && buffer.getSize() > 0 && buffer.getData() != nullptr) {
    m_uart.push(buffer.getData(), buffer.getSize());
  } else if (status == Drv::ByteStreamStatus::OTHER_ERROR) {
    m_uart.setBusError();
  }
  if (this->isConnected_uartRecvReturn_OutputPort(0)) {
    this->uartRecvReturn_out(0, buffer);
  }
}

void GnssDriver::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  m_core.setParams(readParams());
  if (!m_started) { m_core.start(); m_started = true; }
  m_core.tick(t_s);
  this->tlmWrite_HsState(toFppState(m_core.state()));
  this->tlmWrite_LockStage(toFppLock(m_core.lockStage()));
  this->tlmWrite_FaultCount(m_faultCount);
  this->tlmWrite_BadChecksumCount(m_core.badChecksumCount());
  this->tlmWrite_TruncatedCount(m_core.truncatedCount());
  this->tlmWrite_DataAge(m_lastDataAge);
}

void GnssDriver::onFix(const Gnss::GnssFix& f) {
  m_lastDataAge = f.dataAge_s;
  this->tlmWrite_FixSeq(f.seq);
  if (!this->isConnected_fixOut_OutputPort(0)) return;
  this->fixOut_out(0, Gnss::toFpp(f));
}

void GnssDriver::onStateChange(Gnss::DrvState from, Gnss::DrvState to) {
  this->log_ACTIVITY_HI_StateChanged(toFppState(from), toFppState(to));
}

void GnssDriver::onLockStageChange(Gnss::LockStage from, Gnss::LockStage to) {
  this->log_ACTIVITY_HI_LockStageChanged(toFppLock(from), toFppLock(to));
}

void GnssDriver::onFault(Gnss::GnssFault fault, uint32_t count) {
  m_faultCount++;
  this->log_WARNING_HI_Fault(static_cast<U8>(fault), count);
}

void GnssDriver::onFixFailedOpenSky() {
  this->log_WARNING_HI_FixFailedOpenSky();
}

}  // namespace Components
