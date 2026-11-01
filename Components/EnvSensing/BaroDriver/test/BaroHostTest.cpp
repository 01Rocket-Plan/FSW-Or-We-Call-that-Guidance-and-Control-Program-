// BaroDriver 核心 host 測試（BME680 位於 DS／下節；不需 F´）：見 ../../run_host_tests.sh
#include "../Bme680Core.hpp"
#include "../Bme680SimDevice.hpp"
#include <cmath>    // 數學函式（fabs、pow 等）
#include <cstdio>   // 輸入輸出（printf 印測試結果）
#include <vector>   // 可以隨時增刪元素的容器（收集輸出樣本）

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
  CHECK(c.t1 == 26111 && c.p2 == -10423 && c.h1 == 794 && c.h2 == 1019);
  double tf; double t = Bme680Math::compTemp(c, 500000, tf);
  CHECK(t > 0 && t < 50);
  CHECK(std::fabs(Bme680Math::altFromPress(101325, 101325)) < 1e-6);
  CHECK(std::fabs(Bme680Math::altFromPress(89874.6, 101325) - 1000.0) < 2.0);
}

static void testNominal() {
  Rig r; r.dev.setTruth(100000.0, 22.0, 40.0);
  r.run(300);  // 3 s
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.faults.empty());
  CHECK(r.sink.baro.size() >= 140);  // 無氣體插槽，穩定約 50 Hz（扣掉 INIT 幾個 tick）
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
  const EnvOut& e = r.sink.env.back(); std::printf("  env valid=%d h=%f\n", e.valid, e.humidity_pct);
  CHECK(e.valid && std::fabs(e.humidity_pct - 40.0f) < 0.5f);
  // 拿掉氣體插槽後，溫壓間隔固定 20 ms（沒有缺樣本）
  double maxGap = 0; for (size_t i = 1; i < r.sink.baro.size(); i++) maxGap = std::fmax(maxGap, r.sink.baro[i].t_s - r.sink.baro[i - 1].t_s);
  std::printf("  max baro gap %.3f s\n", maxGap);
  CHECK(maxGap < 0.0205);
}

// 加熱器／氣體暫存器（0x5A 加熱電阻、0x64 等待時間、0x70/0x71 氣體控制）永遠不寫
struct SpyBus : IBme680Bus {
  Bme680SimDevice& d; int heaterWrites = 0;
  explicit SpyBus(Bme680SimDevice& dev) : d(dev) {}
  bool writeReg(uint8_t reg, uint8_t v) override {
    if (reg == 0x5A || reg == 0x64 || reg == 0x70 || reg == 0x71) heaterWrites++;
    return d.writeReg(reg, v);
  }
  bool readRegs(uint8_t reg, uint8_t* b, uint8_t n) override { return d.readRegs(reg, b, n); }
};
static void testNoHeater() {
  Bme680SimDevice dev; SpyBus bus(dev); Sink sink; Bme680Core core(bus, sink);
  for (int i = 0; i < 300; i++) { dev.advance(); core.tick(i * 0.01); }
  CHECK(core.state() == DrvState::READY);
  CHECK(bus.heaterWrites == 0);
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
  { Rig r; r.run(100); r.dev.setFault(Bme680SimFault::STUCK); r.run(150);  // 50 筆同值≈100 tick 後進 FAULT（重試在 100 tick 後）
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
  testMath(); testNominal(); testNoHeater(); testAltitude(); testFaults();
  std::printf(g_fail ? "BaroHostTest: %d FAILED\n" : "BaroHostTest: ALL PASSED\n", g_fail);
  return g_fail ? 1 : 0;
}
