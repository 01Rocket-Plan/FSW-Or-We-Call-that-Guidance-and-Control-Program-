// ======================================================================
// \title  ControllerCore.cpp
// \author MCU 組（EdwardWang+Claude）
// \brief  見 ControllerCore.hpp。
//
// 舵權判斷順序（對齊 SSS flight_mode.py `_authority`，先到先擋）：
//   沒模式 → 沒狀態 → 狀態太舊 → imuValid=false → 模式非主動 → demo 固定 → 閉環
// 任何一關擋下：四片 0、lockNeutral=true。
// 發射參考捕捉在「imuValid」那關之後、「模式非主動」那關之前：模式 = BOOST_ACTIVE 且還沒捕捉 → 捕捉。
// ======================================================================
#include "Components/MCU/Controller/ControllerCore.hpp"

#include <cmath>

namespace Mcu {

namespace {
inline float sat(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }
}  // namespace

void ControllerCore::captureLaunchRef(const float q[4]) {
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  // r = R(q_ref)·x̂ = R 的第 1 行：(R00, R10, R20)
  float r0 = 1.0f - 2.0f * (qy * qy + qz * qz);
  float r1 = 2.0f * (qx * qy + qw * qz);
  float r2 = 2.0f * (qx * qz - qw * qy);
  const float n = std::sqrt(r0 * r0 + r1 * r1 + r2 * r2);
  if (!(n > 0.5f)) return;  // q 壞掉（非單位／NaN）→ 不捕捉，下一個合格拍再試
  r0 /= n;
  r1 /= n;
  r2 /= n;
  m_ref[0] = r0;
  m_ref[1] = r1;
  m_ref[2] = r2;
  m_refCaptured = true;
}

void ControllerCore::pd(const float q[4], const float w[3], float& dy, float& dz, float err[2]) const {
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  const float r0 = m_ref[0], r1 = m_ref[1], r2 = m_ref[2];
  // u_b = Rᵀ·r：u_y = R 第 2 行·r，u_z = R 第 3 行·r（r = (0,0,1) 時退回 (R21, R22)，即舊式）
  const float uy = 2.0f * (qx * qy - qw * qz) * r0 + (1.0f - 2.0f * (qx * qx + qz * qz)) * r1 +
                   2.0f * (qy * qz + qw * qx) * r2;
  const float uz = 2.0f * (qx * qz + qw * qy) * r0 + 2.0f * (qy * qz - qw * qx) * r1 +
                   (1.0f - 2.0f * (qx * qx + qy * qy)) * r2;
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
  // Status #8：第一次進 BOOST_ACTIVE 的第一個合格拍，凍結當下姿態當發射參考
  if (m_mode == CtlMode::BOOST_ACTIVE && !m_refCaptured) captureLaunchRef(m_state.q_wxyz);
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
