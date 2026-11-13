// PlatformCore 主機測試（輕量）
#include "Components/MCU/Platform/PlatformCore.hpp"

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

class FakeHal final : public IPlatformHal {
 public:
  ResetCause r = ResetCause::POWER_ON;
  unsigned kicks = 0;
  void kickWatchdog() override { ++kicks; }
  ResetCause resetReason() const override { return r; }
};

int main() {
  std::printf("ST-PLAT-KICK-OVERRUN\n");
  FakeHal hal;
  PlatformCore c(hal);
  double t = 0;
  for (int k = 0; k < 100; ++k) {
    t += 1e-3;
    CHECK(!c.tick(t), "正常 1 ms 不超時");
  }
  CHECK(hal.kicks == 100, "每拍踢一次狗");
  t += 0.005;  // 卡 5 ms
  CHECK(c.tick(t), "5 ms 間隔算超時");
  CHECK(c.stats().overruns == 1, "超時計一次");
  CHECK(c.stats().maxDt_s > 0.0049f, "maxDt 記到");

  std::printf("ST-PLAT-RESET\n");
  CHECK(!c.abnormalReset(), "POWER_ON 正常");
  hal.r = ResetCause::WATCHDOG;
  CHECK(c.abnormalReset(), "WATCHDOG 異常");
  hal.r = ResetCause::BROWNOUT;
  CHECK(c.abnormalReset(), "BROWNOUT 異常");
  hal.r = ResetCause::SOFTWARE;
  CHECK(!c.abnormalReset(), "SOFTWARE 正常");

  if (g_fail) {
    std::printf("PlatformCore: FAIL (%d)\n", g_fail);
    return 1;
  }
  std::printf("PlatformCore: PASS\n");
  return 0;
}
