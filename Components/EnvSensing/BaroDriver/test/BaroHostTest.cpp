// BaroDriver 核心 host 測試（不需 F´）：c++ -std=c++17 -I.. ../Bme680Core.cpp ../Bme680SimDevice.cpp BaroHostTest.cpp
#include "../Bme680Core.hpp"
#include "../Bme680SimDevice.hpp"
#include <cmath>
#include <cstdio>
#include <vector>

using namespace Components;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct Sink : IBme680Sink {
  std::vector<BaroOut> baro; std::vector<EnvOut> env; std::vector<BaroFault> faults;
  void onBaro(const BaroOut& s) override { baro.push_back(s); }
  void onEnv(const EnvOut& s) override { env.push_back(s); }
  void onStateChange(DrvState, DrvState) override {}
  void onFault(BaroFault c, uint32_t) override { faults.push_back(c); }
};

struct Rig {
  Bme680SimDevice dev; Sink sink; Bme680Core core{dev, sink}; double t = 0;
  void run(int n) { for (int i = 0; i < n; i++) { dev.advance(); core.tick(t); t += 0.01; } }
};

static void testMath() {
  uint8_t a[42]; Bme680SimDevice::typicalCoeff(a);
  Bme680Calib c{}; CHECK(Bme680Math::parseCalib(a, c));
  CHECK(c.t1 == 26111 && c.p2 == -10423 && c.h1 == 794 && c.h2 == 1019 && c.resHeatRange == 1);
  double tf; double t = Bme680Math::compTemp(c, 500000, tf);
  CHECK(t > 0 && t < 50);
  CHECK(Bme680Math::gasWaitCode(30) == 30);
  CHECK(Bme680Math::gasWaitCode(100) == (25 + 64));
  CHECK(std::fabs(Bme680Math::altFromPress(101325, 101325)) < 1e-6);
  CHECK(std::fabs(Bme680Math::altFromPress(89874.6, 101325) - 1000.0) < 2.0);
  uint8_t r = Bme680Math::heaterRes(c, 320, 25); CHECK(r > 50 && r < 200);
}

static void testNominal() {
  Rig r; r.dev.setTruth(100000.0, 22.0, 40.0, 80000.0);
  r.run(300);  // 3 s
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.faults.empty());
  CHECK(r.sink.baro.size() > 120);  // 扣掉氣體轉換，約 50 Hz
  std::printf("  baro %zu samples in 3 s, env %zu\n", r.sink.baro.size(), r.sink.env.size());
  CHECK(r.sink.env.size() >= 2 && r.sink.env.size() <= 4);
  for (size_t i = 1; i < r.sink.baro.size(); i++) {
    CHECK(r.sink.baro[i].seq == r.sink.baro[i - 1].seq + 1);
    CHECK(r.sink.baro[i].t_s > r.sink.baro[i - 1].t_s);
  }
  const BaroOut& b = r.sink.baro.back();
  CHECK(b.valid);
  CHECK(std::fabs(b.press_Pa - 100000.0f) < 5.0f);
  CHECK(std::fabs(b.temp_C - 22.0f) < 0.1f);
  CHECK(std::fabs(b.alt_m) < 1.0f);  // p0 取自發射台平均
  CHECK(std::fabs(r.core.p0_Pa() - 100000.0f) < 3.0f);
  CHECK(!r.sink.baro.front().valid);  // 自測期間 valid=false
  const EnvOut& e = r.sink.env.back(); std::printf("  env valid=%d h=%f g=%f\n", e.valid, e.humidity_pct, e.gas_ohm);
  CHECK(e.valid && std::fabs(e.humidity_pct - 40.0f) < 0.5f && std::fabs(e.gas_ohm - 80000.0f) / 80000.0f < 0.05f);
  // 氣體轉換期間 t_s 間隔變大（估測器看得出缺樣本）
  double maxGap = 0; for (size_t i = 1; i < r.sink.baro.size(); i++) maxGap = std::fmax(maxGap, r.sink.baro[i].t_s - r.sink.baro[i - 1].t_s);
  CHECK(maxGap > 0.035);
}

static void testAltitude() {
  Rig r; r.dev.setAltitude(0); r.run(100);
  r.dev.setAltitude(500); r.run(20);
  CHECK(std::fabs(r.sink.baro.back().alt_m - 500.0f) < 3.0f);
}

static void testFaults() {
  { Rig r; r.dev.setFault(Bme680SimFault::BAD_CHIP_ID); r.run(5);
    CHECK(r.core.state() == DrvState::FAULT && !r.sink.faults.empty() && r.sink.faults[0] == BaroFault::BAD_CHIP_ID); }
  { Rig r; r.run(100); CHECK(r.core.state() == DrvState::READY);
    size_t n = r.sink.baro.size(); r.dev.setFault(Bme680SimFault::NACK); r.run(10);
    CHECK(r.core.state() == DrvState::FAULT && r.sink.faults.back() == BaroFault::BUS_ERROR);
    CHECK(r.sink.baro.size() == n);
    r.dev.setFault(Bme680SimFault::NONE); r.run(250);  // 重試後恢復
    CHECK(r.core.state() == DrvState::READY); }
  { Rig r; r.run(100); r.dev.setFault(Bme680SimFault::STUCK); r.run(200);
    CHECK(r.core.state() == DrvState::FAULT && r.sink.faults.back() == BaroFault::STUCK); }
  { Rig r; r.run(100); r.dev.setFault(Bme680SimFault::OUT_OF_RANGE); r.run(30);
    CHECK(r.core.state() == DrvState::FAULT && r.sink.faults.back() == BaroFault::OUT_OF_RANGE);
    for (auto& b : r.sink.baro) CHECK(b.press_Pa > 10000.0f); }
  { Rig r; r.run(100); r.dev.setFault(Bme680SimFault::NO_DATA); r.run(40);
    CHECK(r.core.state() == DrvState::FAULT && r.sink.faults.back() == BaroFault::MEAS_TIMEOUT); }
  { Rig r; r.dev.setFault(Bme680SimFault::NACK); r.run(2000);  // 重試次數有上限
    CHECK(r.core.state() == DrvState::FAULT); }
}

int main() {
  testMath(); testNominal(); testAltitude(); testFaults();
  std::printf(g_fail ? "BaroHostTest: %d FAILED\n" : "BaroHostTest: ALL PASSED\n", g_fail);
  return g_fail ? 1 : 0;
}
