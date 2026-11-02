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
  // Core 每 poll 呼叫一次；BUS 旗標消費後清，避免永久卡 FAULT 路徑以外的重試。
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
  // 滿則停寫：寧可丟尾端，也不在 ISR／匯流排回呼裡阻塞或擴容。
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
  // 每拍重讀：飛行中可經 prmDb 調門檻／間隔而無須重編。
  // rgPeriod_ms=10 對齊 schedIn＝rg100Hz；整數分頻 ticks＝POLL_*_MS/10。
  Fw::ParamValid pv;
  Gnss::GnssCoreParams p;
  p.hdopMax = this->paramGet_HDOP_MAX(pv);
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
  // 只堆積；真正解析在 schedIn→tick→doPoll 的那一次 readBytes（規則：一 poll 一次 bus）。
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
  // 時間來自 F´ Time；Core 用整數 tick 分頻決定本拍是否 poll。
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
  // seq 已在 Core emit 時遞增；此處寫遙測並轉 FPP 結構出 fixOut。
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
  // 含軟故障碼（FIX_FAILED／INIT_ACQ_TIMEOUT）：計數＋事件，但不改變「是否 FAULT 狀態」
  // ——那由 Core 依硬／軟分流決定（C6）。
  m_faultCount++;
  this->log_WARNING_HI_Fault(static_cast<U8>(fault), count);
}

void GnssDriver::onFixFailedOpenSky() {
  // E4 原文一次；字串在 .fpp，此處只觸發。
  this->log_WARNING_HI_FixFailedOpenSky();
}

}  // namespace Components
