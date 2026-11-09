// ======================================================================
// \title  EstimatorCore.cpp
// \author MCU 組（EdwardWang+Claude）
// \brief  見 EstimatorCore.hpp。一拍順序：
//         1. 判斷 IMU 新不新（seq 變了且 valid）
//         2. 墊上未對準 → 累積平均，滿 padAlign_s 後對準
//         3. 已對準 → 姿態積分（墊上加水平修正、估零偏）
//         4. 垂直 KF 預測（加速度）→ 氣壓更新 → 墊上零速更新
//         5. 組 StateOut
// ======================================================================
#include "Components/MCU/Estimator/EstimatorCore.hpp"

#include <cmath>

namespace Mcu {

namespace {

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float norm3(const float v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

}  // namespace

EstimatorCore::EstimatorCore()
    : m_fSum{0, 0, 0},
      m_wSum{0, 0, 0},
      m_nSum(0),
      m_tAlignStart_s(0.0),
      m_q{1, 0, 0, 0},
      m_h(0),
      m_v(0),
      m_P{{1, 0}, {0, 1}},
      m_vE(0),
      m_vN(0),
      m_altRefInit(false),
      m_haveImu(false),
      m_lastImuSeq(0),
      m_lastImuT_s(0.0),
      m_lastBaroSeq(0),
      m_lastBaroT_s(0.0),
      m_haveBaro(false),
      m_wasFresh(false),
      m_seq(0),
      m_aU(0) {}

// ---------------------------------------------------------------- 數學
void EstimatorCore::quatToR(const float q[4], float R[3][3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0][0] = 1 - 2 * (y * y + z * z);
  R[0][1] = 2 * (x * y - w * z);
  R[0][2] = 2 * (x * z + w * y);
  R[1][0] = 2 * (x * y + w * z);
  R[1][1] = 1 - 2 * (x * x + z * z);
  R[1][2] = 2 * (y * z - w * x);
  R[2][0] = 2 * (x * z - w * y);
  R[2][1] = 2 * (y * z + w * x);
  R[2][2] = 1 - 2 * (x * x + y * y);
}

void EstimatorCore::quatMul(const float a[4], const float b[4], float o[4]) {
  const float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  const float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  const float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  const float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  o[0] = w;
  o[1] = x;
  o[2] = y;
  o[3] = z;
}

void EstimatorCore::quatNormalize(float q[4]) {
  const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-9f) {
    q[0] = 1;
    q[1] = q[2] = q[3] = 0;
    return;
  }
  for (int i = 0; i < 4; ++i) q[i] /= n;
  if (q[0] < 0) {
    for (int i = 0; i < 4; ++i) q[i] = -q[i];  // 半球固定，遙測好讀
  }
}

void EstimatorCore::quatFromUp(const float a_body[3], float q[4]) {
  // 找 q 使 R(q)·â = (0,0,1)。最短弧：q = normalize([1 + â·u, â × u])。
  float a[3] = {a_body[0], a_body[1], a_body[2]};
  const float n = norm3(a);
  if (n < 1e-6f) {
    q[0] = 1;
    q[1] = q[2] = q[3] = 0;
    return;
  }
  for (float& c : a) c /= n;
  const float d = a[2];  // â·(0,0,1)
  if (d < -0.9999f) {    // 倒立：繞機體 X 轉 180°
    q[0] = 0;
    q[1] = 1;
    q[2] = q[3] = 0;
    return;
  }
  // â × (0,0,1) = (a_y, −a_x, 0)
  q[0] = 1.0f + d;
  q[1] = a[1];
  q[2] = -a[0];
  q[3] = 0.0f;
  quatNormalize(q);
}

// ---------------------------------------------------------------- 對準
void EstimatorCore::align(const float fMean[3]) {
  quatFromUp(fMean, m_q);
  m_diag.aligned = true;
  m_h = 0;
  m_v = 0;
  m_vE = m_vN = 0;
  m_P[0][0] = m_p.rBaro_m2;
  m_P[0][1] = m_P[1][0] = 0;
  m_P[1][1] = m_p.rZupt_m2ps2;
}

// ---------------------------------------------------------------- 姿態
void EstimatorCore::propagateAttitude(const float w[3], const float f[3], float dt) {
  float wc[3] = {w[0], w[1], w[2]};
  if (m_diag.onPad) {
    // Mahony 水平修正：量到的「上」= f̂；估的「上」在機體 = R 的第 3 列。
    const float fn = norm3(f);
    if (std::fabs(fn - m_p.g_mps2) < m_p.accGate_mps2 && fn > 1e-3f) {
      float R[3][3];
      quatToR(m_q, R);
      const float ub[3] = {R[2][0], R[2][1], R[2][2]};
      const float fm[3] = {f[0] / fn, f[1] / fn, f[2] / fn};
      // e = f̂ × u_b
      const float e[3] = {fm[1] * ub[2] - fm[2] * ub[1], fm[2] * ub[0] - fm[0] * ub[2],
                          fm[0] * ub[1] - fm[1] * ub[0]};
      for (int i = 0; i < 3; ++i) wc[i] += m_p.kLevel_rads * e[i];
    }
  }
  const float th[3] = {wc[0] * dt, wc[1] * dt, wc[2] * dt};
  const float a = norm3(th);
  float dq[4];
  if (a < 1e-9f) {
    dq[0] = 1;
    dq[1] = 0.5f * th[0];
    dq[2] = 0.5f * th[1];
    dq[3] = 0.5f * th[2];
  } else {
    const float s = std::sin(0.5f * a) / a;
    dq[0] = std::cos(0.5f * a);
    dq[1] = th[0] * s;
    dq[2] = th[1] * s;
    dq[3] = th[2] * s;
  }
  float qn[4];
  quatMul(m_q, dq, qn);  // 機體角速度 → 右乘
  for (int i = 0; i < 4; ++i) m_q[i] = qn[i];
  quatNormalize(m_q);
}

// ---------------------------------------------------------------- 垂直 KF
void EstimatorCore::kfPredict(float a_u, float dt) {
  m_h += m_v * dt + 0.5f * a_u * dt * dt;
  m_v += a_u * dt;
  // P = F P Fᵀ + Q，F = [[1,dt],[0,1]]
  const float p00 = m_P[0][0] + dt * (m_P[1][0] + m_P[0][1]) + dt * dt * m_P[1][1];
  const float p01 = m_P[0][1] + dt * m_P[1][1];
  const float p11 = m_P[1][1];
  const float q = m_p.qAcc_m2ps4;
  const float dt2 = dt * dt;
  m_P[0][0] = p00 + q * dt2 * dt2 * 0.25f;
  m_P[0][1] = m_P[1][0] = p01 + q * dt2 * dt * 0.5f;
  m_P[1][1] = p11 + q * dt2;
}

void EstimatorCore::kfUpdateAlt(float z) {
  const float innov = z - m_h;
  if (std::fabs(innov) > m_p.baroGate_m) {
    ++m_diag.baroRejectCount;
    return;
  }
  const float S = m_P[0][0] + m_p.rBaro_m2;
  const float k0 = m_P[0][0] / S, k1 = m_P[1][0] / S;
  m_h += k0 * innov;
  m_v += k1 * innov;
  const float p00 = m_P[0][0], p01 = m_P[0][1], p11 = m_P[1][1];
  m_P[0][0] = (1 - k0) * p00;
  m_P[0][1] = m_P[1][0] = (1 - k0) * p01;
  m_P[1][1] = p11 - k1 * p01;
}

void EstimatorCore::kfUpdateVelZero() {
  const float innov = -m_v;
  const float S = m_P[1][1] + m_p.rZupt_m2ps2;
  const float k0 = m_P[0][1] / S, k1 = m_P[1][1] / S;
  m_h += k0 * innov;
  m_v += k1 * innov;
  const float p00 = m_P[0][0], p01 = m_P[0][1], p11 = m_P[1][1];
  m_P[0][0] = p00 - k0 * p01;
  m_P[0][1] = m_P[1][0] = (1 - k1) * p01;
  m_P[1][1] = (1 - k1) * p11;
}

// ---------------------------------------------------------------- 一拍
void EstimatorCore::step(double now_s, StateOutC& out) {
  // 1. IMU 新鮮度
  const bool newImu = m_imu.valid && (!m_haveImu || m_imu.seq != m_lastImuSeq);
  float dt = 0.0f;
  float wUnb[3] = {0, 0, 0};
  if (newImu) {
    dt = m_haveImu ? static_cast<float>(m_imu.t_s - m_lastImuT_s) : 0.0f;
    dt = clampf(dt, 0.0f, m_p.dtMax_s);
    m_haveImu = true;
    m_lastImuSeq = m_imu.seq;
    m_lastImuT_s = m_imu.t_s;
  }
  const bool fresh = m_haveImu && m_imu.valid && (now_s - m_lastImuT_s) <= m_p.imuStale_s;
  if (m_wasFresh && !fresh) ++m_diag.imuStaleCount;
  m_wasFresh = fresh;

  const float* f = m_imu.acc_mps2;
  for (int i = 0; i < 3; ++i) wUnb[i] = m_imu.gyro_rads[i] - m_diag.gyroBias_rads[i];

  if (newImu) {
    if (!m_diag.aligned) {
      // 2. 墊上對準：只有墊上才累積
      if (m_diag.onPad) {
        if (m_nSum == 0) m_tAlignStart_s = m_imu.t_s;
        for (int i = 0; i < 3; ++i) {
          m_fSum[i] += f[i];
          m_wSum[i] += m_imu.gyro_rads[i];
        }
        ++m_nSum;
        if (m_imu.t_s - m_tAlignStart_s >= m_p.padAlign_s && m_nSum > 0) {
          float fm[3];
          for (int i = 0; i < 3; ++i) {
            fm[i] = m_fSum[i] / static_cast<float>(m_nSum);
            m_diag.gyroBias_rads[i] = m_wSum[i] / static_cast<float>(m_nSum);
          }
          align(fm);
        }
      }
    } else {
      // 3. 姿態；墊上靜止時慢慢追零偏
      if (m_diag.onPad && dt > 0.0f) {
        const float fn = norm3(f);
        if (std::fabs(fn - m_p.g_mps2) < m_p.accGate_mps2 && norm3(wUnb) < m_p.rateGate_rads) {
          const float a = clampf(dt / m_p.gyroBiasTau_s, 0.0f, 1.0f);
          for (int i = 0; i < 3; ++i) m_diag.gyroBias_rads[i] += a * (m_imu.gyro_rads[i] - m_diag.gyroBias_rads[i]);
        }
      }
      if (dt > 0.0f) propagateAttitude(wUnb, f, dt);

      // 4. 比力轉世界、扣重力
      float R[3][3];
      quatToR(m_q, R);
      const float fw[3] = {R[0][0] * f[0] + R[0][1] * f[1] + R[0][2] * f[2],
                           R[1][0] * f[0] + R[1][1] * f[1] + R[1][2] * f[2],
                           R[2][0] * f[0] + R[2][1] * f[1] + R[2][2] * f[2]};
      m_aU = fw[2] - m_p.g_mps2;
      if (dt > 0.0f) {
        kfPredict(m_aU, dt);
        m_vE += fw[0] * dt;
        m_vN += fw[1] * dt;
      }
    }
  }

  // 4b. 氣壓
  const bool newBaro = m_baro.valid && (!m_haveBaro || m_baro.seq != m_lastBaroSeq);
  if (newBaro) {
    const float dtB = m_haveBaro ? clampf(static_cast<float>(m_baro.t_s - m_lastBaroT_s), 0.0f, 1.0f) : 0.0f;
    m_haveBaro = true;
    m_lastBaroSeq = m_baro.seq;
    m_lastBaroT_s = m_baro.t_s;
    if (!m_altRefInit) {
      m_diag.altRef_m = m_baro.alt_m;
      m_altRefInit = true;
    } else if (m_diag.onPad) {
      // 墊上氣壓基準慢慢追（天氣漂移）；離台後凍結
      const float a = clampf(dtB / m_p.altRefTau_s, 0.0f, 1.0f);
      m_diag.altRef_m += a * (m_baro.alt_m - m_diag.altRef_m);
    }
    if (m_diag.aligned) kfUpdateAlt(m_baro.alt_m - m_diag.altRef_m);
  }
  if (m_diag.aligned && m_diag.onPad && newImu) {
    kfUpdateVelZero();
    m_vE = m_vN = 0;
  }

  // 5. 輸出
  out.seq = ++m_seq;
  out.t_s = now_s;
  for (int i = 0; i < 4; ++i) out.q_wxyz[i] = m_q[i];
  for (int i = 0; i < 3; ++i) out.rate_body_rads[i] = wUnb[i];
  float R[3][3];
  quatToR(m_q, R);
  out.tilt_rad = std::acos(clampf(R[2][2], -1.0f, 1.0f));  // 機體 Z（鼻）離世界 +U
  out.vel_enu_mps[0] = m_vE;
  out.vel_enu_mps[1] = m_vN;
  out.vel_enu_mps[2] = m_v;
  out.alt_m = m_h;
  out.baroAge_s = m_haveBaro ? static_cast<float>(now_s - m_lastBaroT_s) : 1.0e6f;
  out.accAxial_mps2 = m_diag.aligned ? m_aU : 0.0f;
  out.imuValid = fresh && m_diag.aligned;
  out.velValid = out.imuValid && out.baroAge_s <= m_p.baroMaxAge_s;
  out.gyroSat = m_haveImu && m_imu.gyroSat;
}

}  // namespace Mcu
