// ======================================================================
// \title  PlatformCore.cpp
// \author MCU 組（EdwardWang+Claude）
// ======================================================================
#include "Components/MCU/Platform/PlatformCore.hpp"

namespace Mcu {

bool PlatformCore::tick(double now_s) {
  bool over = false;
  if (m_have) {
    const float dt = static_cast<float>(now_s - m_last_s);
    m_st.lastDt_s = dt;
    if (dt > m_st.maxDt_s) m_st.maxDt_s = dt;
    if (dt > m_p.overrun_s) {
      ++m_st.overruns;
      over = true;
    }
  }
  m_have = true;
  m_last_s = now_s;
  ++m_st.ticks;
  m_hal->kickWatchdog();  // 走到這裡＝這拍整條 1 kHz 鏈都跑完了
  return over;
}

bool PlatformCore::abnormalReset() const {
  const ResetCause r = m_hal->resetReason();
  return r == ResetCause::WATCHDOG || r == ResetCause::BROWNOUT || r == ResetCause::PANIC;
}

}  // namespace Mcu
