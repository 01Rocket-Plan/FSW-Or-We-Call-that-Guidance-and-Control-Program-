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
  c.setMode(true);

  qPadThen(1, 0.0f, q);  // 直立
  c.pushState(st(1.0, q, w0));
  c.step(1.0, o);
  CHECK(!o.lockNeutral && o.reason == HoldReason::NONE, "閉環");
  for (float f : o.fin_rad) CHECK(std::fabs(f) < 1e-6f, "直立 → 四片 0");

  qPadThen(1, 0.1f, q);  // 繞機體 +Y（俯仰）歪 0.1 rad → 要 −Y 力矩
  c.pushState(st(1.001, q, w0));
  c.step(1.001, o);
  CHECK(o.fin_rad[0] < 0, "+Y 歪 → δy < 0");
  CHECK(std::fabs(o.fin_rad[0] + 1.2f * std::sin(0.1f)) < 1e-4f, "δy = −kp·sin(0.1)");
  CHECK(std::fabs(o.fin_rad[1]) < 1e-6f, "δz = 0");
  CHECK(o.fin_rad[2] == -o.fin_rad[0] && o.fin_rad[3] == -o.fin_rad[1], "混控 (δy,δz,−δy,−δz)");

  qPadThen(2, 0.1f, q);  // 繞機體 +Z（偏航）歪 → 要 −Z 力矩
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
  c.setMode(true);
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
  qPadThen(1, 1.0f, q);
  const float w[3] = {0, 3.0f, 0};
  c.setMode(true);
  c.pushState(st(1.0, q, w));
  FinOut o;
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
    c.setMode(true);
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::NO_STATE, "沒狀態 → 鎖");
  }
  {
    ControllerCore c;
    c.setMode(true);
    c.pushState(st(1.0, q, w));
    c.step(1.021, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::STATE_STALE, "狀態 21 ms 舊 → 鎖");
    c.step(1.019, o);
    CHECK(!o.lockNeutral, "19 ms 仍閉環");
  }
  {
    ControllerCore c;
    c.setMode(true);
    c.pushState(st(1.0, q, w, false));
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::IMU_INVALID, "imuValid=false → 鎖");
  }
  {
    ControllerCore c;
    c.setMode(false);
    c.pushState(st(1.0, q, w));
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::MODE_INACTIVE, "非主動模式 → 鎖");
  }
  {
    ControllerCore c;
    ControllerParams p;
    p.demo50mFixed = true;
    c.setParams(p);
    c.setMode(true);
    c.pushState(st(1.0, q, w));
    c.step(1.0, o);
    CHECK(o.lockNeutral && o.reason == HoldReason::DEMO_FIXED, "demo 固定翼 → 舵恆 0");
    for (float f : o.fin_rad) CHECK(f == 0.0f, "四片 0");
  }
}

int main() {
  testSignAndMix();
  testDamping();
  testSaturation();
  testAuthorityGates();
  if (g_fail) {
    std::printf("ControllerCore: FAIL (%d)\n", g_fail);
    return 1;
  }
  std::printf("ControllerCore: PASS\n");
  return 0;
}
