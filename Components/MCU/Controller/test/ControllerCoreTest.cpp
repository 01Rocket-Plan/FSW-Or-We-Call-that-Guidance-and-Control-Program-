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

//! 繞機體軸 axis（0=X,1=Y）轉 a rad 的四元數（機體→世界）
static void qAxis(int axis, float a, float q[4]) {
  q[0] = std::cos(0.5f * a);
  q[1] = axis == 0 ? std::sin(0.5f * a) : 0.0f;
  q[2] = axis == 1 ? std::sin(0.5f * a) : 0.0f;
  q[3] = 0.0f;
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
  qAxis(0, 0.1f, q);  // 箭繞 +X 歪了 0.1 rad → 要往 −X 轉回
  c.setMode(true);
  c.pushState(st(1.0, q, w0));
  FinOut o;
  c.step(1.0, o);
  CHECK(!o.lockNeutral && o.reason == HoldReason::NONE, "閉環");
  CHECK(o.fin_rad[0] < 0, "+X 歪 → δx < 0（要 −X 力矩）");
  CHECK(std::fabs(o.fin_rad[0] + 1.2f * std::sin(0.1f)) < 1e-4f, "δx = −kp·sin(0.1)");
  CHECK(std::fabs(o.fin_rad[1]) < 1e-6f, "δy = 0");
  CHECK(o.fin_rad[2] == -o.fin_rad[0] && o.fin_rad[3] == -o.fin_rad[1], "混控 (δx,δy,−δx,−δy)");

  qAxis(1, 0.1f, q);  // 繞 +Y 歪
  c.pushState(st(1.001, q, w0));
  c.step(1.001, o);
  CHECK(o.fin_rad[1] < 0 && std::fabs(o.fin_rad[0]) < 1e-6f, "+Y 歪 → δy < 0");
}

static void testDamping() {
  std::printf("ST-CTL-DAMP\n");
  ControllerCore c;
  const float q[4] = {1, 0, 0, 0};
  const float w[3] = {0.4f, -0.2f, 5.0f};  // 滾轉 5 rad/s 不應影響
  c.setMode(true);
  c.pushState(st(1.0, q, w));
  FinOut o;
  c.step(1.0, o);
  CHECK(std::fabs(o.fin_rad[0] - (-0.25f * 0.4f)) < 1e-6f, "δx = −kd·ωx");
  CHECK(std::fabs(o.fin_rad[1] - (0.25f * 0.2f)) < 1e-6f, "δy = −kd·ωy");
}

static void testSaturation() {
  std::printf("ST-CTL-SAT\n");
  ControllerCore c;
  float q[4];
  qAxis(0, 1.0f, q);
  const float w[3] = {3.0f, 0, 0};
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
