#include "Components/Gnss/GnssDriver/GnssDriver.hpp"
#include "Components/Gnss/GnssDriver/GnssToFpp.hpp"
#include <cstring>

namespace Components {

static RocketTypes::DrvState toFppState(Gnss::DrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

int32_t GnssUartAdapter::readBytes(uint8_t* buf, uint32_t max) {
  // 匯流排錯誤：回傳 <0，核心進 FAULT（每個 tick 最多呼叫一次）
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
    if (m_size >= CAP) break;  // 滿了就丟最舊策略改為丟新資料，避免覆蓋未讀
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
  p.selftestTicks = this->paramGet_SELFTEST_TICKS(pv);
  p.noDataWarnTicks = this->paramGet_NODATA_WARN_TICKS(pv);
  p.noDataTimeoutTicks = this->paramGet_NODATA_TIMEOUT_TICKS(pv);
  p.noDataFaultTicks = this->paramGet_NODATA_FAULT_TICKS(pv);
  p.stuckTicks = this->paramGet_STUCK_TICKS(pv);
  p.fixStaleAge_s = this->paramGet_FIX_STALE_AGE_S(pv);
  return p;
}

void GnssDriver::uartRecv_handler(FwIndexType, Fw::Buffer& buffer, const Drv::ByteStreamStatus& status) {
  // UART 驅動推入資料；不在這裡解析，等 schedIn 讓核心一次 readBytes
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
  // 時間一律取自 F´ 時間服務；核心不讀系統時間
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;

  m_core.setParams(readParams());
  if (!m_started) { m_core.start(); m_started = true; }  // OFF -> INIT

  m_core.tick(t_s);  // 內部一次 UART readBytes + 非阻塞狀態機

  this->tlmWrite_HsState(toFppState(m_core.state()));
  this->tlmWrite_FaultCount(m_faultCount);
  this->tlmWrite_BadChecksumCount(m_core.badChecksumCount());
  this->tlmWrite_TruncatedCount(m_core.truncatedCount());
  this->tlmWrite_DataAge(m_lastDataAge);
  this->tlmWrite_NoDataTicks(m_core.noDataTicks());
  this->tlmWrite_FixLost(m_core.fixLostLatched() ? 1 : 0);
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

void GnssDriver::onFault(Gnss::GnssFault fault, uint32_t count) {
  m_faultCount++;
  this->log_WARNING_HI_Fault(static_cast<U8>(fault), count);
}

}  // namespace Components
