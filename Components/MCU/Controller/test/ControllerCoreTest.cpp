// ControllerCore 主機測試（輕量；CONTRIBUTING 第 10 條）
#include "Components/MCU/Controller/ControllerCore.hpp"

#include <cmath>
#include <cstdio>

using namespace Mcu;

static int g_fail = 0;
#define CHECK(c, msg)                                      \
  do {                                                     \
    if (!(c)) {                                            \
      std::printf("  FAIL %s (line %d)\n", msg, __LINE__); \
      ++g_fail;                                            \
    }                                                      \
  } while (0)

//! 墊上直立（機體 X 指 +U）＝繞世界 N 轉 −90°；再繞「機體」axis（1=Y,2=Z）轉 a rad
static void qPadThen(int axis, float a, float q[4]) {
  const float h = std::sqrt(0.5f);
  const float p[4] = {h, 0.0f, -h, 0.0f};  // R(p)·x̂ = (0,0,1)
  const float d[4] = {std::cos(0.5f * a), 0.0f, axis == 1 ? std::sin(0.5f * a) : 0.0f,
                      axis == 2 ? std::sin(0.5f * a) : 0.0f};
  q[0] = p[0] * d[0] - p[1] * d[1] - p[2] * d[2] - p[3] * d[3];
  q[1] = p[0] * d[1] + p[1] * d[0] + p[2] * d[3] - p[3] * d[2];
  q[2] = p[0] * d[2] - p[1] * d[3] + p[2] * d[0] + p[3] * d[1];
  q[3] = p[0] * d[3] + p[1] * d[2] - p[2] * d[1] + p[3] * d[0];
}

//! 四元數乘法 out = a ⊗ b
static void qmul(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

//! 墊上直立 → 繞機體 Y 俯仰 pitch → 再繞機體 Z 偏航 yaw（rad）
static void qPadPitchYaw(float pitch, float yaw, float q[4]) {
  float p[4];
  qPadThen(1, pitch, p);
  const float d[4] = {std::cos(0.5f * yaw), 0.0f, 0.0f, std::sin(0.5f * yaw)};
  qmul(p, d, q);
}

static const float kDeg = 3.14159265f / 180.0f;

//! 改之前（目標寫死世界垂直）的 PD，拿來比對「參考 = 垂直」時完全一樣
static void legacyPd(const ControllerParams& p, const float q[4], const float w[3], float& dy, float& dz) {
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  const float uy = 2.0f * (qy * qz + qw * qx);
  const float uz = 1.0f - 2.0f * (qx * qx + qy * qy);
  const float ey = -uz, ez = uy;
  auto sat = [](float x, float l) { return x > l ? l : (x < -l ? -l : x); };
  dy = sat(p.kp * ey - p.kd * w[1], p.finMax_rad);
  dz = sat(p.kp * ez - p.kd * w[2], p.finMax_rad);
}

static StateIn st(double t, const float q[4], const float w[3], bool valid = true) {
  StateIn s;
  s.t_s = t;
  for (int i = 0; i < 4; ++i) s.q_wxyz[i] = q[i];
  for (int i = 0; i < 3; ++i) s.rate_body_rads[i] = w[i];
  s.imuValid = valid;
  return s;
}

static void testSignAndMix() {
  std::printf("ST-CTL-SIGN-MIX\n");
  ControllerCore c;
  float q[4];
  const float w0[3] = {0, 0, 0};
  FinOut o;
  c.setMode(CtlMode::BOOST_ACTIVE);

  qPadThen(1, 0.0f, q);  // 直立：第一拍 BOOST 捕捉 → 發射參考 = 垂直（以下誤差都相對這個參考）
  c.pushState(st(1.0, q, w0));
  c.step(1.0, o);
  CHECK(!o.lockNeutral && o.reason == HoldReason::NONE, "閉環");
  CHECK(c.launchRefCaptured() && std::fabs(c.launchRef()[2] - 1.0f) < 1e-6f, "參考 = 垂直 (0,0,1)");
  for (float f : o.fin_rad) CHECK(std::fabs(f) < 1e-6f, "直立 → 四片 0");

  qPadThen(1, 0.1f, q);  // 相對參考（垂直）繞機體 +Y（俯仰）歪 0.1 rad → 要 −Y 力矩
  c.pushState(st(1.001, q, w0));
  c.step(1.001, o);
  CHECK(o.fin_rad[0] < 0, "+Y 歪 → δy < 0");
  CHECK(std::fabs(o.fin_rad[0] + 1.2f * std::sin(0.1f)) < 1e-4f, "δy = −kp·sin(0.1)");
  CHECK(std::fabs(o.fin_rad[1]) < 1e-6f, "δz = 0");
  CHECK(o.fin_rad[2] == -o.fin_rad[0] && o.fin_rad[3] == -o.fin_rad[1], "混控 (δy,δz,−δy,−δz)");

  qPadThen(2, 0.1f, q);  // 相對參考（垂直）繞機體 +Z（偏航）歪 → 要 −Z 力矩
  c.pushState(st(1.002, q, w0));
  c.step(1.002, o);
  CHECK(o.fin_rad[1] < 0 && std::fabs(o.fin_rad[0]) < 1e-6f, "+Z 歪 → δz < 0");
  CHECK(std::fabs(o.fin_rad[1] + 1.2f * std::sin(0.1f)) < 1e-4f, "δz = −kp·sin(0.1)");
}

static void testDamping() {
  std::printf("ST-CTL-DAMP\n");
  ControllerCore c;
  float q[4];
  qPadThen(1, 0.0f, q);
  const float w[3] = {5.0f, 0.4f, -0.2f};  // 滾轉 ωx 5 rad/s 不應影響
  c.setMode(CtlMode::BOOST_ACTIVE);
  c.pushState(st(1.0, q, w));
  FinOut o;
  c.step(1.0, o);
  CHECK(std::fabs(o.fin_rad[0] - (-0.25f * 0.4f)) < 1e-6f, "δy = −kd·ωy");
  CHECK(std::fabs(o.fin_rad[1] - (0.25f * 0.2f)) < 1e-6f, "δz = −kd·ωz");
}

static void testSaturation() {
  std::printf("ST-CTL-SAT\n");
  ControllerCore c;
  float q[4];
  const float w0[3] = {0, 0, 0};
  FinOut o;
  c.setMode(CtlMode::BOOST_ACTIVE);
  qPadThen(1, 0.0f, q);  // 先在直立捕捉 → 參考 = 垂直
  c.pushState(st(0.999, q, w0));
  c.step(0.999, o);
  qPadThen(1, 1.0f, q);
  const float w[3] = {0, 3.0f, 0};
  c.pushState(st(1.0, q, w));
  c.step(1.0, o);
  CHECK(std::fabs(o.fin_rad[0] + c.params().finMax_rad) < 1e-6f, "限幅 −30°");
}

static void testAuthorityGates() {
  std::printf("ST-CTL-AUTH\n");
  const float q[4] = {1, 0, 0, 0};
  const float w[3] = {0, 0, 0};
  FinOut o;
  {
    ControllerCore c;
    c.pushState(st(1.0, q, w));
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::NO_MODE, "沒模式 → 鎖");
  }
  {
    ControllerCore c;
    c.setMode(CtlMode::BOOST_ACTIVE);
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::NO_STATE, "沒狀態 → 鎖");
  }
  {
    ControllerCore c;
    c.setMode(CtlMode::BOOST_ACTIVE);
    c.pushState(st(1.0, q, w));
    c.step(1.021, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::STATE_STALE, "狀態 21 ms 舊 → 鎖");
    c.step(1.019, o);
    CHECK(!o.lockNeutral, "19 ms 仍閉環");
  }
  {
    ControllerCore c;
    c.setMode(CtlMode::BOOST_ACTIVE);
    c.pushState(st(1.0, q, w, false));
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::IMU_INVALID, "imuValid=false → 鎖");
  }
  {
    ControllerCore c;
    c.setMode(CtlMode::INACTIVE);
    c.pushState(st(1.0, q, w));
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::MODE_INACTIVE, "非主動模式 → 鎖");
  }
  {
    ControllerCore c;
    ControllerParams p;
    p.demo50mFixed = true;
    c.setParams(p);
    c.setMode(CtlMode::BOOST_ACTIVE);
    c.pushState(st(1.0, q, w));
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::DEMO_FIXED, "demo 固定翼 → 舵恆 0");
    for (float f : o.fin_rad) CHECK(f == 0.0f, "四片 0");
  }
}

// ---------------------------------------------------------------- 發射參考（Status #8）
//! (a) 傾 5° 發射：BOOST 第一拍捕捉，角速度 0 → 四片 ≈ 0；沒捕捉（舊行為＝垂直）會打 −kp·sin5°
static void testLaunchRefHold() {
  std::printf("ST-CTL-LREF-HOLD\n");
  const float w0[3] = {0, 0, 0};
  float q[4];
  qPadThen(1, 5.0f * kDeg, q);  // 導軌繞機體 Y 傾 5°
  FinOut o;
  {
    ControllerCore c;
    c.setMode(CtlMode::BOOST_ACTIVE);
    c.pushState(st(1.0, q, w0));
    c.step(1.0, o);
    CHECK(!o.lockNeutral && o.reason == HoldReason::NONE, "閉環");
    CHECK(c.launchRefCaptured(), "BOOST 第一拍捕捉");
    for (float f : o.fin_rad) CHECK(std::fabs(f) < 1e-5f, "傾 5°＝參考 → 四片 ≈ 0");
    // r = 機體 X 在世界的方向：離 +U 5°
    CHECK(std::fabs(std::acos(c.launchRef()[2]) - 5.0f * kDeg) < 1e-4f, "r 離 +U 5°");
    for (int k = 1; k <= 50; ++k) {  // 姿態不變 → 一直 0
      c.pushState(st(1.0 + 0.001 * k, q, w0));
      c.step(1.0 + 0.001 * k, o);
    }
    for (float f : o.fin_rad) CHECK(std::fabs(f) < 1e-5f, "持續 50 拍 ≈ 0");
  }
  {
    // 對照：COAST 但從沒捕捉 → 退回垂直目標（舊行為）
    ControllerCore c;
    c.setMode(CtlMode::COAST);
    c.pushState(st(1.0, q, w0));
    c.step(1.0, o);
    CHECK(!c.launchRefCaptured(), "COAST 不捕捉");
    CHECK(std::fabs(o.fin_rad[0] + 1.2f * std::sin(5.0f * kDeg)) < 1e-4f, "沒參考 → 垂直目標 δy = −kp·sin5°");
  }
}

//! (b) 參考 5° 後再歪 ±2°（俯仰、偏航）→ 拉回參考，不是拉回垂直
static void testLaunchRefPullBack() {
  std::printf("ST-CTL-LREF-PULL\n");
  const float w0[3] = {0, 0, 0};
  const float kp = 1.2f;
  float q[4];
  FinOut o;
  ControllerCore c;
  c.setMode(CtlMode::BOOST_ACTIVE);
  qPadThen(1, 5.0f * kDeg, q);
  c.pushState(st(1.0, q, w0));
  c.step(1.0, o);

  qPadThen(1, 7.0f * kDeg, q);  // 往外多歪 +2°
  c.pushState(st(1.001, q, w0));
  c.step(1.001, o);
  CHECK(o.fin_rad[0] < 0, "7°（參考 +2°）→ δy < 0");
  CHECK(std::fabs(o.fin_rad[0] + kp * std::sin(2.0f * kDeg)) < 1e-4f, "δy = −kp·sin2°（不是 −kp·sin7°）");
  CHECK(std::fabs(o.fin_rad[1]) < 1e-5f, "δz ≈ 0");

  qPadThen(1, 3.0f * kDeg, q);  // 往垂直回 −2°：舊式會繼續推向垂直（δy<0），新式要拉回 5°（δy>0）
  c.pushState(st(1.002, q, w0));
  c.step(1.002, o);
  CHECK(o.fin_rad[0] > 0, "3°（參考 −2°）→ δy > 0（拉回參考，不是垂直）");
  CHECK(std::fabs(o.fin_rad[0] - kp * std::sin(2.0f * kDeg)) < 1e-4f, "δy = +kp·sin2°");
  {
    float dy, dz;
    legacyPd(c.params(), q, w0, dy, dz);
    CHECK(dy < 0, "對照：舊式（垂直目標）在 3° 時 δy < 0");
  }

  qPadPitchYaw(5.0f * kDeg, 2.0f * kDeg, q);  // 偏航 +2°
  c.pushState(st(1.003, q, w0));
  c.step(1.003, o);
  CHECK(std::fabs(o.fin_rad[1] + kp * std::sin(2.0f * kDeg)) < 1e-4f, "偏航 +2° → δz = −kp·sin2°");
  CHECK(std::fabs(o.fin_rad[0]) < 1e-4f, "δy ≈ 0");
  qPadPitchYaw(5.0f * kDeg, -2.0f * kDeg, q);  // 偏航 −2°
  c.pushState(st(1.004, q, w0));
  c.step(1.004, o);
  CHECK(std::fabs(o.fin_rad[1] - kp * std::sin(2.0f * kDeg)) < 1e-4f, "偏航 −2° → δz = +kp·sin2°");
  CHECK(o.fin_rad[2] == -o.fin_rad[0] && o.fin_rad[3] == -o.fin_rad[1], "混控不變");
}

//! (c) 參考 = 垂直 → 跟改之前（寫死垂直）的 PD 逐點一樣（含阻尼、限幅）
static void testLaunchRefVerticalEqualsLegacy() {
  std::printf("ST-CTL-LREF-VERT\n");
  const float w0[3] = {0, 0, 0};
  float q[4];
  FinOut o;
  ControllerCore c;
  c.setMode(CtlMode::BOOST_ACTIVE);
  qPadThen(1, 0.0f, q);
  c.pushState(st(1.0, q, w0));
  c.step(1.0, o);
  CHECK(std::fabs(c.launchRef()[0]) < 1e-6f && std::fabs(c.launchRef()[1]) < 1e-6f, "r ≈ (0,0,1)");
  const float pitches[] = {-1.0f, -0.3f, -0.05f, 0.0f, 0.02f, 0.1f, 0.7f};
  const float yaws[] = {-0.4f, 0.0f, 0.03f, 0.25f};
  const float ws[][3] = {{0, 0, 0}, {5.0f, 0.4f, -0.2f}, {0, 3.0f, -3.0f}};
  int n = 0;
  double t = 1.0;
  float maxDiff = 0.0f;
  for (float p : pitches)
    for (float y : yaws)
      for (const auto& w : ws) {
        qPadPitchYaw(p, y, q);
        t += 0.001;
        c.pushState(st(t, q, w));
        c.step(t, o);
        float dy, dz;
        legacyPd(c.params(), q, w, dy, dz);
        maxDiff = std::fmax(maxDiff, std::fmax(std::fabs(o.fin_rad[0] - dy), std::fabs(o.fin_rad[1] - dz)));
        ++n;
      }
  std::printf("  %d 點，與舊式最大差 %.2e rad\n", n, static_cast<double>(maxDiff));
  CHECK(maxDiff < 1e-6f, "參考 = 垂直 → 與舊式 PD 一致（< 1e-6 rad）");
}

//! (d) BOOST 前不捕捉；imuValid／狀態新鮮才捕捉；COAST／ABORT 保持；回墊上（重新 ARMED）清掉、下次重新捕捉
static void testLaunchRefCaptureRules() {
  std::printf("ST-CTL-LREF-ARM\n");
  const float w0[3] = {0, 0, 0};
  float q5[4], q3[4], q8[4];
  qPadThen(1, 5.0f * kDeg, q5);
  qPadThen(1, 3.0f * kDeg, q3);
  qPadThen(1, 8.0f * kDeg, q8);
  FinOut o;
  ControllerCore c;
  double t = 1.0;
  auto tick = [&](CtlMode m, const float q[4], bool valid) {
    t += 0.001;
    c.setMode(m);
    c.pushState(st(t, q, w0, valid));
    c.step(t, o);
  };
  tick(CtlMode::PRE_LAUNCH, q5, true);  // 墊上／ARMED
  CHECK(!c.launchRefCaptured() && o.lockNeutral && o.reason == HoldReason::MODE_INACTIVE, "墊上不捕捉、不閉環");
  tick(CtlMode::INACTIVE, q5, true);  // LIFTOFF／BOOST_PASSIVE
  CHECK(!c.launchRefCaptured() && o.lockNeutral, "LIFTOFF／BOOST_PASSIVE 不捕捉");
  tick(CtlMode::BOOST_ACTIVE, q3, false);  // 進 BOOST 那拍 imuValid=false
  CHECK(!c.launchRefCaptured() && o.reason == HoldReason::IMU_INVALID, "imuValid=false → 不捕捉");
  {
    // 狀態太舊也不捕捉
    t += 0.030;
    c.step(t, o);
    CHECK(!c.launchRefCaptured() && o.reason == HoldReason::STATE_STALE, "狀態太舊 → 不捕捉");
  }
  tick(CtlMode::BOOST_ACTIVE, q5, true);  // BOOST 內第一個合格拍
  CHECK(c.launchRefCaptured() && !o.lockNeutral, "BOOST 內第一個合格拍捕捉");
  for (float f : o.fin_rad) CHECK(std::fabs(f) < 1e-5f, "捕捉拍四片 ≈ 0");
  const float r2 = c.launchRef()[2];
  tick(CtlMode::BOOST_ACTIVE, q8, true);  // 之後姿態變了，參考不跟著動
  CHECK(std::fabs(c.launchRef()[2] - r2) < 1e-7f && o.fin_rad[0] < 0, "BOOST 後續拍不重捕");
  tick(CtlMode::COAST, q8, true);
  CHECK(std::fabs(c.launchRef()[2] - r2) < 1e-7f && !o.lockNeutral && o.fin_rad[0] < 0, "COAST 沿用參考");
  tick(CtlMode::INACTIVE, q8, true);  // ABORT 等
  CHECK(c.launchRefCaptured() && o.lockNeutral, "ABORT 不閉環、參考保持");
  tick(CtlMode::PRE_LAUNCH, q8, true);  // 回墊上／重新 ARMED
  CHECK(!c.launchRefCaptured() && std::fabs(c.launchRef()[2] - 1.0f) < 1e-7f, "回 PAD_IDLE／ARMED → 清掉參考");
  tick(CtlMode::BOOST_ACTIVE, q8, true);  // 下一次 BOOST 重新捕捉（8°）
  CHECK(c.launchRefCaptured() && std::fabs(std::acos(c.launchRef()[2]) - 8.0f * kDeg) < 1e-4f, "重新 ARMED 後重新捕捉");
  for (float f : o.fin_rad) CHECK(std::fabs(f) < 1e-5f, "新參考下四片 ≈ 0");
  {
    ControllerCore fresh;  // 重開機 = core 重建
    CHECK(!fresh.launchRefCaptured(), "重開機沒有參考");
  }
}

int main() {
  testSignAndMix();
  testDamping();
  testSaturation();
  testAuthorityGates();
  testLaunchRefHold();
  testLaunchRefPullBack();
  testLaunchRefVerticalEqualsLegacy();
  testLaunchRefCaptureRules();
  if (g_fail) {
    std::printf("ControllerCore: FAIL (%d)\n", g_fail);
    return 1;
  }
  std::printf("ControllerCore: PASS\n");
  return 0;
}
