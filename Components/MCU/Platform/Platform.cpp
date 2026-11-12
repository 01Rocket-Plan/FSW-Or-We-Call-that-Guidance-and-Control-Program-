// ======================================================================
// \title  Platform.cpp
// \author MCU 組（EdwardWang+Claude）
// ======================================================================
#include "Components/MCU/Platform/Platform.hpp"

namespace Mcu {

namespace {
constexpr U32 kTlmDivider = 1000;      // 1 kHz → 1 Hz
constexpr U32 kMaxOverrunEvents = 20;  // 超時事件上限（之後只看遙測，避免事件洪水）

ResetReason toFpp(Mcu::ResetCause r) {
  return ResetReason(static_cast<ResetReason::T>(static_cast<U8>(r)));
}
}  // namespace

Platform::Platform(const char* compName)
    : PlatformComponentBase(compName), m_core(m_defaultHal), m_tlmDiv(0), m_overrunEvents(0), m_bootReported(false) {}

Platform::~Platform() {}

void Platform::loadParams() {
  Fw::ParamValid v;
  PlatformParams p;
  p.loopPeriod_s = this->paramGet_loopPeriod_s(v);
  p.overrun_s = this->paramGet_overrun_s(v);
  m_core.setParams(p);
}

void Platform::parameterUpdated(FwPrmIdType) { loadParams(); }
void Platform::parametersLoaded() { loadParams(); }

void Platform::CLEAR_MAX_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.clearMax();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Platform::schedIn_handler(FwIndexType, U32) {
  if (!m_bootReported) {
    m_bootReported = true;
    this->log_ACTIVITY_HI_Boot(toFpp(m_core.resetReason()));
    if (m_core.abnormalReset()) this->log_WARNING_HI_AbnormalReset(toFpp(m_core.resetReason()));
  }
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  if (m_core.tick(t_s) && m_overrunEvents < kMaxOverrunEvents) {
    ++m_overrunEvents;
    this->log_WARNING_LO_LoopOverrun(m_core.stats().lastDt_s);
  }
  if (++m_tlmDiv >= kTlmDivider) {
    m_tlmDiv = 0;
    const PlatformStats& st = m_core.stats();
    this->tlmWrite_resetReason(toFpp(m_core.resetReason()));
    this->tlmWrite_ticks(st.ticks);
    this->tlmWrite_overruns(st.overruns);
    this->tlmWrite_maxLoopDt_s(st.maxDt_s);
  }
}

}  // namespace Mcu
