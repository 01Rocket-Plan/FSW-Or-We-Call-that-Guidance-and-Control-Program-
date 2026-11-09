// ======================================================================
// \title  ControllerCore.cpp
// \author MCU 組（EdwardWang+Claude）
// \brief  見 ControllerCore.hpp。
//
// 舵權判斷順序（對齊 SSS flight_mode.py `_authority`，先到先擋）：
//   沒模式 → 沒狀態 → 狀態太舊 → imuValid=false → 模式非主動 → demo 固定 → 閉環
// 任何一關擋下：四片 0、lockNeutral=true。
// ======================================================================
#include "Components/MCU/Controller/ControllerCore.hpp"

namespace Mcu {

namespace {
inline float sat(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }
}  // namespace

void ControllerCore::pd(const float q[4], const float w[3], float& dy, float& dz, float err[2]) const {
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  // u_b = Rᵀ·(0,0,1) = R 的第 3 列：(R20, R21, R22)
  const float uy = 2.0f * (qy * qz + qw * qx);
  const float uz = 1.0f - 2.0f * (qx * qx + qy * qy);
  // e = x̂ × u_b = (0, −u_z, u_y)
  err[0] = -uz;  // 俯仰（繞 Y）
  err[1] = uy;   // 偏航（繞 Z）
  dy = sat(m_p.kp * err[0] - m_p.kd * w[1], m_p.finMax_rad);
  dz = sat(m_p.kp * err[1] - m_p.kd * w[2], m_p.finMax_rad);
}

void ControllerCore::step(double now_s, FinOut& out) {
  out.t_s = now_s;
  out.seq = ++m_seq;
  out.err_rad[0] = out.err_rad[1] = 0.0f;
  for (float& f : out.fin_rad) f = 0.0f;
  out.lockNeutral = true;

  if (!m_haveMode) {
    out.reason = HoldReason::NO_MODE;
    return;
  }
  if (!m_haveState) {
    out.reason = HoldReason::NO_STATE;
    return;
  }
  if (now_s - m_state.t_s > m_p.stateStale_s) {
    out.reason = HoldReason::STATE_STALE;
    return;
  }
  if (!m_state.imuValid) {
    out.reason = HoldReason::IMU_INVALID;
    return;
  }
  if (!m_activeMode) {
    out.reason = HoldReason::MODE_INACTIVE;
    return;
  }
  if (m_p.demo50mFixed) {
    out.reason = HoldReason::DEMO_FIXED;
    return;
  }

  float dy = 0, dz = 0;
  pd(m_state.q_wxyz, m_state.rate_body_rads, dy, dz, out.err_rad);
  out.fin_rad[0] = dy;
  out.fin_rad[1] = dz;
  out.fin_rad[2] = -dy;
  out.fin_rad[3] = -dz;
  out.lockNeutral = false;
  out.reason = HoldReason::NONE;
}

}  // namespace Mcu
