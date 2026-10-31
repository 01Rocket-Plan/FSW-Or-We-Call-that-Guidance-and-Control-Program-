// UsDht22 核心 host 測試（不需 F´）
#include "../Dht22Core.hpp"
#include "../Dht22SimIo.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace EnvSensing;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct Sink : IDht22Sink {
  std::vector<HumTempOut> s; std::vector<DhtFault> f;
  void onSample(const HumTempOut& x) override { s.push_back(x); }
  void onStateChange(DhtState, DhtState) override {}
  void onFault(DhtFault c, uint32_t) override { f.push_back(c); }
};
struct Rig { Dht22SimIo io; Sink sink; Dht22Core core{io, sink}; double t = 0;
  void run(int n) { for (int i = 0; i < n; i++) { core.tick(t); t += 1.0; } } };

static void testDecode() {
  // datasheet 範例：濕度 65.2 %（0x028C）、溫度 35.1 °C（0x015F）、checksum 0xEE
  uint8_t ex[5] = {0x02, 0x8C, 0x01, 0x5F, 0xEE};
  CHECK(Dht22Decode::checksumOk(ex));
  float t, h; Dht22Decode::toPhysical(ex, t, h);
  CHECK(std::fabs(t - 35.1f) < 1e-4f && std::fabs(h - 65.2f) < 1e-4f);
  uint8_t neg[5] = {0x02, 0x8C, 0x80, 0x65, 0x00}; neg[4] = static_cast<uint8_t>(neg[0] + neg[1] + neg[2] + neg[3]);
  Dht22Decode::toPhysical(neg, t, h); CHECK(std::fabs(t + 10.1f) < 1e-4f);
  uint16_t highs[42]; highs[0] = 30; highs[1] = 80;
  for (int i = 0; i < 40; i++) highs[i + 2] = (ex[i / 8] & (0x80 >> (i % 8))) ? 70 : 27;
  uint8_t out[5]; CHECK(Dht22Decode::bitsFromHighs(highs, 42, 48, out));
  for (int i = 0; i < 5; i++) CHECK(out[i] == ex[i]);
  CHECK(!Dht22Decode::bitsFromHighs(highs, 39, 48, out));
}

static void testNominal() {
  Rig r; r.io.setTruth(-5.3f, 72.4f); r.run(41);  // INIT 1 tick + 20 筆
  CHECK(r.core.state() == DhtState::READY && r.sink.f.empty());
  std::printf("  dht %zu samples in 41 s\n", r.sink.s.size());
  CHECK(r.sink.s.size() == 20);
  for (size_t i = 1; i < r.sink.s.size(); i++) {
    CHECK(r.sink.s[i].seq == r.sink.s[i - 1].seq + 1);
    CHECK(std::fabs(r.sink.s[i].t_s - r.sink.s[i - 1].t_s - 2.0) < 1e-9);  // 每 2 s
  }
  CHECK(!r.sink.s.front().valid && r.sink.s.back().valid);
  const HumTempOut& s = r.sink.s.back();
  CHECK(std::fabs(s.temp_K - (273.15f - 5.3f)) < 0.15f && std::fabs(s.humidity_frac - 0.724f) < 1e-3f);
}

static void testFaults() {
  { Rig r; r.run(10); size_t n = r.sink.s.size(); r.io.setFault(Dht22SimFault::BAD_CHECKSUM); r.run(6);
    CHECK(r.core.state() == DhtState::FAULT && r.sink.f.back() == DhtFault::CHECKSUM && r.sink.s.size() == n);
    r.io.setFault(Dht22SimFault::NONE); r.run(20); CHECK(r.core.state() == DhtState::READY); }
  { Rig r; r.run(10); r.io.setFault(Dht22SimFault::NO_RESPONSE); r.run(6);
    CHECK(r.core.state() == DhtState::FAULT && r.sink.f.back() == DhtFault::NO_RESPONSE); }
  { Rig r; r.run(10); r.io.setFault(Dht22SimFault::OUT_OF_RANGE); r.run(6);
    CHECK(r.core.state() == DhtState::FAULT && r.sink.f.back() == DhtFault::OUT_OF_RANGE); }
  { Rig r; r.run(10); r.io.setFault(Dht22SimFault::SHORT_FRAME); r.run(6);
    CHECK(r.core.state() == DhtState::FAULT && r.sink.f.back() == DhtFault::CHECKSUM); }
  { Rig r; Dht22Config c; c.stuckN = 5; r.core.setConfig(c); r.run(10); r.io.setFault(Dht22SimFault::STUCK); r.run(12);
    CHECK(r.core.state() == DhtState::FAULT && r.sink.f.back() == DhtFault::STUCK); }
  { Rig r; r.io.setFault(Dht22SimFault::NO_RESPONSE); r.run(500); CHECK(r.core.state() == DhtState::FAULT); }
}

int main() {
  testDecode(); testNominal(); testFaults();
  if (g_fail) std::printf("Dht22HostTest: %d FAILED\n", g_fail); else std::printf("Dht22HostTest: ALL PASSED\n");
  return g_fail ? 1 : 0;
}
