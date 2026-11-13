// ActuatorMgrCore 主機測試（輕量；CONTRIBUTING 第 10 條）
#include "Components/MCU/ActuatorMgr/ActuatorMgrCore.hpp"

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

static FinIn fin(double t, float a, float b, float c, float d, bool lock = false) {
  FinIn f;
  f.t_s = t;
  f.fin_rad[0] = a;
  f.fin_rad[1] = b;
  f.fin_rad[2] = c;
  f.fin_rad[3] = d;
  f.lockNeutral = lock;
  return f;
}

//! 跑 n 拍（每拍 1 ms，每拍都送同一個 FinCmd 新時間戳）
static void run(ActuatorMgrCore& m, double& t, int n, float a, ServoOut& o, SepOut& s) {
  for (int k = 0; k < n; ++k) {
    t += 1e-3;
    m.pushFin(fin(t, a, -a, -a, a));
    m.step(t, o, s);
  }
}

static void testMapAndRate() {
  std::printf("ST-ACT-MAP-RATE\n");
  ActuatorMgrCore m;
  ServoOut o;
  SepOut s;
  double t = 0;
  run(m, t, 1, 0.1f, o, s);
  CHECK(!o.locked, "有新 FinCmd → 不鎖");
  CHECK(o.pulse_us[0] == 1500, "第一拍 dt 未知 → 不動");
  run(m, t, 1, 0.1f, o, s);
  // 速率限制：10.47 rad/s × 1 ms = 0.01047 rad
  CHECK(std::fabs((o.pulse_us[0] - 1500) / 318.30989f - 0.01047f) < 0.002f, "第二拍被速率限制");
  run(m, t, 50, 0.1f, o, s);
  CHECK(std::abs(static_cast<int>(o.pulse_us[0]) - (1500 + 32)) <= 1, "穩態 0.1 rad → 1532 µs");
  CHECK(std::abs(static_cast<int>(o.pulse_us[1]) - (1500 - 32)) <= 1, "負向 → 1468 µs");
  CHECK(o.rateLimitCount > 0, "有記速率限制");
}

static void testLimitsAndClamp() {
  std::printf("ST-ACT-LIMIT\n");
  ActuatorMgrCore m;
  ActuatorMgrParams p;
  p.rateMax_rads = 1e6f;  // 關掉速率限制，單看限幅
  m.setParams(p);
  ServoOut o;
  SepOut s;
  double t = 0;
  run(m, t, 2, 2.0f, o, s);  // 2 rad 遠超 30°
  CHECK(std::abs(static_cast<int>(o.pulse_us[0]) - (1500 + 167)) <= 1, "限幅到 +30° → 1667 µs");
  p.gear = 4.0f;  // 連桿比放大到撞 µs 硬限
  m.setParams(p);
  run(m, t, 2, 0.5f, o, s);
  CHECK(o.pulse_us[0] == 2000 && o.pulse_us[1] == 1000, "µs 硬限 1000–2000");
  CHECK(o.clampCount > 0, "記硬限次數");
}

static void testDirTrim() {
  std::printf("ST-ACT-DIR-TRIM\n");
  ActuatorMgrCore m;
  ActuatorMgrParams p;
  p.rateMax_rads = 1e6f;
  p.dir[0] = -1.0f;
  p.trim_rad[1] = 0.05f;
  m.setParams(p);
  ServoOut o;
  SepOut s;
  double t = 0;
  run(m, t, 2, 0.1f, o, s);
  CHECK(std::abs(static_cast<int>(o.pulse_us[0]) - (1500 - 32)) <= 1, "dir=−1 反向");
  CHECK(std::abs(static_cast<int>(o.pulse_us[1]) - (1500 + static_cast<int>(std::lround((-0.1f + 0.05f) * 318.31f)))) <= 1,
        "trim 疊加");
}

static void testLockSources() {
  std::printf("ST-ACT-LOCK\n");
  ServoOut o;
  SepOut s;
  {
    ActuatorMgrCore m;
    m.step(0.001, o, s);
    CHECK(o.locked && o.reason == LockReason::NO_CMD, "沒收過 FinCmd → 鎖");
    CHECK(o.lockNeutral, "進入鎖定那一拍 lockNeutral=true");
    m.step(0.002, o, s);
    CHECK(o.locked && !o.lockNeutral, "之後維持中位但不再重複 lockNeutral");
    for (int i = 0; i < 4; ++i) CHECK(o.pulse_us[i] == 1500, "中位 1500");
  }
  {
    ActuatorMgrCore m;
    double t = 0;
    run(m, t, 100, 0.2f, o, s);
    m.pushFin(fin(t + 0.001, 0.2f, 0, 0, 0, true));
    m.step(t + 0.001, o, s);
    CHECK(o.locked && o.reason == LockReason::CMD_LOCK && o.pulse_us[0] == 1500, "FinCmd.lockNeutral → 立刻回中位");
  }
  {
    ActuatorMgrCore m;
    double t = 0;
    run(m, t, 10, 0.2f, o, s);
    m.step(t + 0.021, o, s);
    CHECK(o.locked && o.reason == LockReason::CMD_STALE, "FinCmd 21 ms 沒更新 → 鎖");
  }
  {
    ActuatorMgrCore m;
    double t = 0;
    run(m, t, 10, 0.2f, o, s);
    m.setUv(1);  // UV_WARN
    run(m, t, 1, 0.2f, o, s);
    CHECK(!o.locked, "UV_WARN 不鎖");
    m.setUv(2);  // UV_SHED
    run(m, t, 1, 0.2f, o, s);
    CHECK(o.locked && o.reason == LockReason::UNDERVOLT, "UV_SHED → 鎖");
    m.setUv(0);
    run(m, t, 1, 0.2f, o, s);
    CHECK(!o.locked && std::abs(static_cast<int>(o.pulse_us[0]) - 1503) <= 1, "解鎖從 0 平滑起步（速率限制）");
  }
}

static void testSep() {
  std::printf("ST-ACT-SEP\n");
  ServoOut o;
  SepOut s;
  {
    ActuatorMgrCore m;
    SepIn c;
    c.seq = 1;
    c.channelMask = 0x07;
    c.valid = false;
    CHECK(!m.pushSep(c, 1.0), "valid=false 丟棄");
    c.valid = true;
    m.setUv(3);  // UV_CRITICAL：分離仍要走
    CHECK(m.pushSep(c, 1.0), "收下");
    m.step(1.0, o, s);
    CHECK(s.send && s.channelMask == 0x07, "stagger=0 → 三顆一起");
    m.step(1.001, o, s);
    CHECK(!s.send, "只送一次");
    CHECK(!m.pushSep(c, 1.002), "同 seq 再來 → 丟棄");
  }
  {
    ActuatorMgrCore m;
    ActuatorMgrParams p;
    p.sepStagger_s = 0.010f;
    m.setParams(p);
    SepIn c;
    c.seq = 5;
    c.channelMask = 0x07;
    c.valid = true;
    m.pushSep(c, 2.0);
    uint8_t seen = 0;
    int sends = 0;
    double firstT = -1, lastT = -1;
    for (int k = 0; k <= 30; ++k) {
      const double tt = 2.0 + k * 1e-3;
      m.step(tt, o, s);
      if (s.send) {
        CHECK((s.channelMask & (s.channelMask - 1)) == 0, "錯開時一次只放一顆");
        seen |= s.channelMask;
        ++sends;
        if (firstT < 0) firstT = tt;
        lastT = tt;
      }
    }
    CHECK(sends == 3 && seen == 0x07, "三顆都放了");
    CHECK(std::fabs((lastT - firstT) - 0.020) < 1.5e-3, "首末相隔 2×10 ms");
  }
}

int main() {
  testMapAndRate();
  testLimitsAndClamp();
  testDirTrim();
  testLockSources();
  testSep();
  if (g_fail) {
    std::printf("ActuatorMgrCore: FAIL (%d)\n", g_fail);
    return 1;
  }
  std::printf("ActuatorMgrCore: PASS\n");
  return 0;
}
