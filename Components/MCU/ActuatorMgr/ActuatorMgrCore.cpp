// ======================================================================
// \title  ActuatorMgrCore.cpp
// \author MCU 組（EdwardWang+Claude）
// \brief  見 ActuatorMgrCore.hpp。
// ======================================================================
#include "Components/MCU/ActuatorMgr/ActuatorMgrCore.hpp"

#include <cmath>

namespace Mcu {

namespace {
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
}  // namespace

uint16_t ActuatorMgrCore::toPulse(int ch, float servoAngle_rad, bool& clamped) const {
  (void)ch;
  const float us = static_cast<float>(m_p.neutral_us) + servoAngle_rad * m_p.usPerRad;
  const float lo = static_cast<float>(m_p.pulseMin_us), hi = static_cast<float>(m_p.pulseMax_us);
  clamped = (us < lo) || (us > hi);
  return static_cast<uint16_t>(std::lround(clampf(us, lo, hi)));
}

bool ActuatorMgrCore::pushSep(const SepIn& c, double now_s) {
  if (!c.valid || (m_haveSepSeq && c.seq <= m_lastSepSeq)) {
    ++m_sepDrop;
    return false;
  }
  m_haveSepSeq = true;
  m_lastSepSeq = c.seq;
  const uint8_t req = static_cast<uint8_t>(c.channelMask & 0x0Fu);
  if ((req & ~m_sepPending) != 0 && m_sepPending == 0) m_sepNext_s = now_s;  // 佇列空 → 立刻開始
  m_sepPending = static_cast<uint8_t>(m_sepPending | req);
  return true;
}

void ActuatorMgrCore::step(double now_s, ServoOut& o, SepOut& sep) {
  // ---------------- 時間
  float dt = m_firstStep ? 0.0f : static_cast<float>(now_s - m_lastT_s);
  dt = clampf(dt, 0.0f, m_p.dtMax_s);
  m_firstStep = false;
  m_lastT_s = now_s;

  // ---------------- 鎖中位判斷（先到先擋）
  LockReason why = LockReason::NONE;
  if (!m_haveFin) {
    why = LockReason::NO_CMD;
  } else if (m_uv >= m_p.uvLockLevel) {
    why = LockReason::UNDERVOLT;
  } else if (now_s - m_fin.t_s > m_p.finStale_s) {
    why = LockReason::CMD_STALE;
  } else if (m_fin.lockNeutral) {
    why = LockReason::CMD_LOCK;
  }
  const bool locked = (why != LockReason::NONE);

  o.t_s = now_s;
  o.seq = ++m_seq;
  o.locked = locked;
  o.reason = why;
  o.lockNeutral = locked && !m_wasLocked;  // 只在進入鎖定那一拍
  m_wasLocked = locked;

  for (int i = 0; i < kFinCount; ++i) {
    float servoAngle;
    if (locked) {
      m_theta[i] = 0.0f;  // 解鎖後從 0 平滑起步
      servoAngle = m_p.trim_rad[i];
    } else {
      const float target = clampf(m_fin.fin_rad[i], -m_p.finMax_rad, m_p.finMax_rad);
      const float step = m_p.rateMax_rads * dt;
      const float d = target - m_theta[i];
      if (std::fabs(d) > step) {
        m_theta[i] += (d > 0 ? step : -step);
        ++m_rateLim;
      } else {
        m_theta[i] = target;
      }
      servoAngle = m_p.dir[i] * m_p.gear * m_theta[i] + m_p.trim_rad[i];
    }
    bool clamped = false;
    o.pulse_us[i] = toPulse(i, servoAngle, clamped);
    if (clamped) ++m_clamp;
  }
  o.clampCount = m_clamp;
  o.rateLimitCount = m_rateLim;

  // ---------------- 分離
  sep.send = false;
  if (m_sepPending != 0 && now_s >= m_sepNext_s) {
    uint8_t mask = m_sepPending;
    if (m_p.sepStagger_s > 0.0f) {
      mask = static_cast<uint8_t>(m_sepPending & (~m_sepPending + 1u));  // 最低位
      m_sepNext_s = now_s + m_p.sepStagger_s;
    }
    m_sepPending = static_cast<uint8_t>(m_sepPending & ~mask);
    sep.send = true;
    sep.t_s = now_s;
    sep.seq = ++m_sepOutSeq;
    sep.channelMask = mask;
  }
}

}  // namespace Mcu
