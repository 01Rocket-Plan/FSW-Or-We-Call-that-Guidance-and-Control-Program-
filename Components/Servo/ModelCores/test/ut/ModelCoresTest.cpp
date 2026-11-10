// =============================================================================
// ModelCoresTest.cpp
//
// Host unit tests for Sg90Core, Mg996rCore, Sg90SimDevice, Mg996rSimDevice.
// No F' dependency. Plain asserts; builds with -std=c++14 -Wall -Wextra -Werror.
// =============================================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "Components/Servo/ModelCores/Mg996rCore.hpp"
#include "Components/Servo/ModelCores/Mg996rSimDevice.hpp"
#include "Components/Servo/ModelCores/Sg90Core.hpp"
#include "Components/Servo/ModelCores/Sg90SimDevice.hpp"

namespace {

int g_failures = 0;

#define EXPECT_TRUE(cond)                                                        \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
      ++g_failures;                                                              \
    }                                                                            \
  } while (0)

#define EXPECT_NEAR(a, b, tol)                                                   \
  do {                                                                           \
    const double _a = static_cast<double>(a);                                    \
    const double _b = static_cast<double>(b);                                    \
    const double _t = static_cast<double>(tol);                                  \
    if (std::fabs(_a - _b) > _t) {                                               \
      std::fprintf(stderr, "FAIL %s:%d  |%.6f - %.6f| > %.6f\n", __FILE__,       \
                   __LINE__, _a, _b, _t);                                        \
      ++g_failures;                                                              \
    }                                                                            \
  } while (0)

using namespace Components;

void testClampSg90() {
  Sg90Core core;
  bool clamped = false;

  EXPECT_TRUE(core.clampPulseUs(900, clamped) == 1000);
  EXPECT_TRUE(clamped == true);

  EXPECT_TRUE(core.clampPulseUs(2100, clamped) == 2000);
  EXPECT_TRUE(clamped == true);

  EXPECT_TRUE(core.clampPulseUs(1500, clamped) == 1500);
  EXPECT_TRUE(clamped == false);

  EXPECT_TRUE(core.clampPulseUs(1000, clamped) == 1000);
  EXPECT_TRUE(clamped == false);

  EXPECT_TRUE(core.clampPulseUs(2000, clamped) == 2000);
  EXPECT_TRUE(clamped == false);
}

void testClampMg996r() {
  Mg996rCore core;
  bool clamped = false;

  EXPECT_TRUE(core.clampPulseUs(900, clamped) == 1000);
  EXPECT_TRUE(clamped == true);

  EXPECT_TRUE(core.clampPulseUs(2100, clamped) == 2000);
  EXPECT_TRUE(clamped == true);

  EXPECT_TRUE(core.clampPulseUs(1500, clamped) == 1500);
  EXPECT_TRUE(clamped == false);

  EXPECT_TRUE(core.clampPulseUs(1000, clamped) == 1000);
  EXPECT_TRUE(clamped == false);
}

void testMapRoundTripSg90() {
  Sg90Core core;
  // Neutral <-> 0 deg
  EXPECT_NEAR(core.pulseToAngleDeg(1500), 0.0f, 1e-3f);
  EXPECT_NEAR(core.angleToPulseUs(0.0f), 1500.0f, 0.5f);

  // Min/Max end-stops (SG90 travel 360 -> +/-180)
  EXPECT_NEAR(core.pulseToAngleDeg(1000), -180.0f, 1.0f);
  EXPECT_NEAR(core.pulseToAngleDeg(2000), +180.0f, 1.0f);
  EXPECT_TRUE(core.angleToPulseUs(-180.0f) == 1000);
  EXPECT_TRUE(core.angleToPulseUs(+180.0f) == 2000);
  // ±30°（尾翼飽和）= ±83 µs
  EXPECT_TRUE(core.angleToPulseUs(+30.0f) == 1583);

  // Saturate beyond end-stops
  EXPECT_TRUE(core.angleToPulseUs(-200.0f) == 1000);
  EXPECT_TRUE(core.angleToPulseUs(+200.0f) == 2000);
}

void testMapRoundTripMg996r() {
  Mg996rCore core;
  EXPECT_NEAR(core.pulseToAngleDeg(1500), 0.0f, 1e-3f);
  EXPECT_NEAR(core.angleToPulseUs(0.0f), 1500.0f, 0.5f);

  // MG996R travel 360 -> +/-180
  EXPECT_NEAR(core.pulseToAngleDeg(1000), -180.0f, 1.0f);
  EXPECT_NEAR(core.pulseToAngleDeg(2000), +180.0f, 1.0f);
  EXPECT_TRUE(core.angleToPulseUs(-180.0f) == 1000);
  EXPECT_TRUE(core.angleToPulseUs(+180.0f) == 2000);
  EXPECT_TRUE(core.angleToPulseUs(-200.0f) == 1000);
  EXPECT_TRUE(core.angleToPulseUs(+200.0f) == 2000);
}

void testSg90Slew() {
  Sg90SimDevice dev;
  EXPECT_NEAR(dev.angleDeg(0), 0.0f, 1e-3f);

  // Command step neutral -> max end-stop.
  dev.setCommandPulseUs(0, 2000);
  // 0.05 s: 600 deg/s * 0.05 = 30 deg expected.
  const float dt = 0.05f;
  dev.tick(dt);
  EXPECT_NEAR(dev.angleDeg(0), 30.0f, 2.0f);

  // Enough time to reach end-stop (180 deg @ 600 deg/s = 0.30 s): another 0.3 s.
  for (int i = 0; i < 30; ++i) {
    dev.tick(0.01f);
  }
  EXPECT_NEAR(dev.angleDeg(0), 180.0f, 1e-2f);
  EXPECT_NEAR(dev.measuredPulseUs(0), 2000.0f, 1.0f);

  // Stays at end-stop.
  dev.tick(0.1f);
  EXPECT_NEAR(dev.angleDeg(0), 180.0f, 1e-2f);
}

void testMg996rSlew() {
  Mg996rSimDevice dev;
  EXPECT_NEAR(dev.angleDeg(0), 0.0f, 1e-3f);

  // Step neutral -> max end-stop; end-stop for MG996R = +180 deg (360 travel).
  dev.setCommandPulseUs(0, 2000);
  // 0.05 s: 353 deg/s * 0.05 = 17.65 deg expected.
  dev.tick(0.05f);
  EXPECT_NEAR(dev.angleDeg(0), 17.65f, 2.0f);

  // Enough time to reach end-stop.
  for (int i = 0; i < 50; ++i) {  // 180 deg @ 353 deg/s = 0.51 s
    dev.tick(0.01f);
  }
  EXPECT_NEAR(dev.angleDeg(0), 180.0f, 1e-2f);

  // Stays there.
  dev.tick(0.1f);
  EXPECT_NEAR(dev.angleDeg(0), 180.0f, 1e-2f);
}

void testDtZeroNoOp() {
  Sg90SimDevice sg;
  sg.setCommandPulseUs(1, 2000);
  sg.tick(0.02f);
  const float before = sg.angleDeg(1);
  sg.tick(0.0f);
  EXPECT_NEAR(sg.angleDeg(1), before, 1e-6f);

  Mg996rSimDevice mg;
  mg.setCommandPulseUs(2, 2000);
  mg.tick(0.02f);
  const float before_mg = mg.angleDeg(2);
  mg.tick(0.0f);
  EXPECT_NEAR(mg.angleDeg(2), before_mg, 1e-6f);

  // Negative dt also no-op.
  sg.tick(-0.5f);
  EXPECT_NEAR(sg.angleDeg(1), before, 1e-6f);
}

void testMultiChannelIsolation() {
  Sg90SimDevice sg;
  sg.setCommandPulseUs(0, 2000);
  sg.setCommandPulseUs(1, 1500);
  sg.tick(0.1f);
  EXPECT_TRUE(sg.angleDeg(0) > 1.0f);
  EXPECT_NEAR(sg.angleDeg(1), 0.0f, 1e-6f);

  Mg996rSimDevice mg;
  mg.setCommandPulseUs(0, 2000);
  mg.setCommandPulseUs(1, 1000);
  mg.tick(0.1f);
  EXPECT_TRUE(mg.angleDeg(0) > 1.0f);
  // ch1 commanded to min (lock/fire-low) → negative angle; must not track ch0
  EXPECT_TRUE(mg.angleDeg(1) < -1.0f);
}

void testOutOfRangeChannelIgnored() {
  Sg90SimDevice sg;
  sg.setCommandPulseUs(99, 2000);  // ignored
  sg.tick(0.1f);
  // All channels remain at neutral.
  for (uint8_t ch = 0; ch < Sg90SimDevice::kMaxChannels; ++ch) {
    EXPECT_NEAR(sg.angleDeg(ch), 0.0f, 1e-6f);
  }

  Mg996rSimDevice mg;
  mg.setCommandPulseUs(99, 2000);
  mg.tick(0.1f);
  for (uint8_t ch = 0; ch < Mg996rSimDevice::kMaxChannels; ++ch) {
    EXPECT_NEAR(mg.angleDeg(ch), 0.0f, 1e-6f);
  }
}

void testSimMeasuredPulseTracksCommand() {
  Sg90SimDevice sg;
  sg.setCommandPulseUs(0, 2000);
  // Settle.
  for (int i = 0; i < 100; ++i) {
    sg.tick(0.02f);
  }
  EXPECT_NEAR(sg.measuredPulseUs(0), 2000.0f, 1.0f);
}

}  // namespace

int main() {
  testClampSg90();
  testClampMg996r();
  testMapRoundTripSg90();
  testMapRoundTripMg996r();
  testSg90Slew();
  testMg996rSlew();
  testDtZeroNoOp();
  testMultiChannelIsolation();
  testOutOfRangeChannelIgnored();
  testSimMeasuredPulseTracksCommand();

  if (g_failures != 0) {
    std::fprintf(stderr, "\n%d test failure(s)\n", g_failures);
    return EXIT_FAILURE;
  }
  std::printf("All ModelCores tests passed.\n");
  return EXIT_SUCCESS;
}
