// T1~T8：真 GnssCore（組長）＋ GnssSimDevice，tick = rg1Hz（1 s）
// 行為（v0.2 READY 每 tick 輸出）：dataAge_s = now - last new GGA；截斷只計數+事件、不進 FAULT
#include "Components/Gnss/GnssDriverSim/GnssSimDevice.hpp"
#include "Components/Gnss/GnssDriver/GnssCore.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace Gnss;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)
static const double kDeg = 3.14159265358979323846 / 180.0;

struct Sink : IGnssSink {
  std::vector<GnssFix> fixes; std::vector<GnssFault> faults; std::vector<DrvState> states;
  void onFix(const GnssFix& f) override { fixes.push_back(f); }
  void onStateChange(DrvState, DrvState to) override { states.push_back(to); }
  void onFault(GnssFault f, uint32_t) override { faults.push_back(f); }
  size_t nFault(GnssFault f) const { size_t k = 0; for (auto x : faults) if (x == f) k++; return k; }
};
struct Rig {
  GnssSimDevice dev; Sink sink; GnssCore core{dev, sink}; double t = 0;
  Rig() { const float v[3] = {10.f, 5.f, 2.f}; dev.setTrajectory(24.0 * kDeg, 121.0 * kDeg, 50.0, v); core.start(); }
  void run(int n) { for (int i = 0; i < n; i++) { t += 1.0; dev.advance(1.0); core.tick(t); } }
};

static void t1_readyValid() {
  Rig r; r.run(20);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() >= 15);
  for (auto& f : r.sink.fixes) CHECK(f.valid);  // 只在 READY 輸出且 valid
  Rig q; q.dev.setFault(GnssSimFault::NO_DATA); q.run(10);
  CHECK(q.core.state() == DrvState::FAULT && q.sink.fixes.empty());
  const GnssFix& f = r.sink.fixes.back(); double tl, tn, ta; r.dev.truthAt(r.t - 1.0, tl, tn, ta);
  CHECK(std::fabs(f.lat_rad - tl) * 6378137.0 < 15.0 && std::fabs(f.altMsl_m - ta) < 25.0);
  CHECK(f.fixType == FixType::FIX_3D && f.velHValid && std::fabs(f.velEnu_mps[0] - 10.f) < 1.f);
}
static void t2_velUNaN() {
  Rig r; r.run(10);
  for (auto& f : r.sink.fixes) { CHECK(std::isnan(f.velEnu_mps[2])); CHECK(!f.velUValid); }
}
static void t3_fix2d() {
  Rig r; r.run(10); r.dev.setFault(GnssSimFault::FIX_2D_ONLY); r.run(5);
  const GnssFix& f = r.sink.fixes.back(); CHECK(f.fixType == FixType::FIX_2D && !f.posValid);
}
static void t4_hdop() {
  Rig r; r.dev.setHdopBias(3.0f); r.run(20); size_t hi = 0, lo = 0;
  for (auto& f : r.sink.fixes) { if (f.fixType != FixType::FIX_3D) continue;
    if (f.hdop > 2.5f) { CHECK(!f.posValid); hi++; } else { CHECK(f.posValid); lo++; } }
  std::printf("  hdop>2.5: %zu, <=2.5: %zu\n", hi, lo);
  CHECK(hi > 0);  // bias 讓超門檻可直接測到
  // 再把門檻調嚴
  GnssCoreParams p; p.hdopMax = 0.5f; r.core.setParams(p); r.dev.setHdopBias(0.f); size_t n0 = r.sink.fixes.size(); r.run(5);
  for (size_t i = n0; i < r.sink.fixes.size(); i++) {
    if (r.sink.fixes[i].fixType == FixType::FIX_3D) CHECK(!r.sink.fixes[i].posValid);
  }
}
static void t5_seq() {
  // READY 每 tick 輸出；短 NO_DATA 期間仍輸出且 seq 連續；進 FAULT 後停止；恢復後續編
  Rig r; r.run(10); uint32_t s = r.sink.fixes.back().seq; size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(3);  // 仍 READY，繼續輸出
  CHECK(r.sink.fixes.size() == n + 3);
  CHECK(r.sink.fixes.back().seq == s + 3);
  r.run(5);  // 再 5 tick → 總無資料 >=5 → FAULT，之後無輸出
  CHECK(r.core.state() == DrvState::FAULT);
  size_t nFault = r.sink.fixes.size();
  uint32_t sFault = r.sink.fixes.back().seq;
  r.run(3);
  CHECK(r.sink.fixes.size() == nFault);  // FAULT 中不輸出
  r.dev.setFault(GnssSimFault::NONE); r.run(15);
  CHECK(r.core.state() == DrvState::READY && r.sink.fixes.size() > nFault);
  CHECK(r.sink.fixes[nFault].seq == sFault + 1);  // 不跳號
  for (size_t i = 1; i < r.sink.fixes.size(); i++) CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);
}
static void t6_dataAge() {
  // NO_DATA 期間（仍 READY）dataAge_s 單調遞增
  Rig r; r.run(10); size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(3);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() >= n + 3);
  float a0 = r.sink.fixes[n].dataAge_s, a1 = r.sink.fixes[n + 1].dataAge_s, a2 = r.sink.fixes[n + 2].dataAge_s;
  std::printf("  dataAge during NO_DATA: %.2f %.2f %.2f\n", a0, a1, a2);
  CHECK(a1 > a0 && a2 > a1);
  CHECK(a2 >= 2.0f);
}
static void t7_badChecksum() {
  Rig r; r.run(10); r.dev.setFault(GnssSimFault::BAD_CHECKSUM); r.run(5);
  CHECK(r.core.badChecksumCount() >= 4 && r.sink.nFault(GnssFault::BAD_CHECKSUM) >= 4);
  r.dev.setFault(GnssSimFault::NONE); size_t n = r.sink.fixes.size(); r.run(10);
  CHECK(r.core.state() == DrvState::READY && r.sink.fixes.size() > n);
}
static void t8_truncStuck() {
  // 截斷：計數 + TRUNCATED 事件，不進 FAULT，可恢復
  { Rig r; r.run(10); r.dev.setFault(GnssSimFault::TRUNCATED); r.run(5);
    std::printf("  trunc count %u, TRUNCATED faults %zu, state %d\n",
                r.core.truncatedCount(), r.sink.nFault(GnssFault::TRUNCATED), (int)r.core.state());
    CHECK(r.core.truncatedCount() >= 4 && r.sink.nFault(GnssFault::TRUNCATED) >= 4);
    CHECK(r.core.state() == DrvState::READY);  // 截斷不進 FAULT
    r.dev.setFault(GnssSimFault::NONE); r.run(10); CHECK(r.core.state() == DrvState::READY); }
  { Rig r; r.run(10); r.dev.setFault(GnssSimFault::STUCK); r.run(8);
    CHECK(r.core.state() == DrvState::FAULT && r.sink.nFault(GnssFault::STUCK) >= 1);
    r.dev.setFault(GnssSimFault::NONE); r.run(10); CHECK(r.core.state() == DrvState::READY); }
  { Rig r; r.run(10); r.dev.setFault(GnssSimFault::NO_DATA); r.run(8);
    CHECK(r.core.state() == DrvState::FAULT && r.sink.nFault(GnssFault::NO_DATA) >= 1);
    r.dev.setFault(GnssSimFault::NONE); r.run(10); CHECK(r.core.state() == DrvState::READY); }
}

int main() {
  t1_readyValid(); t2_velUNaN(); t3_fix2d(); t4_hdop(); t5_seq(); t6_dataAge(); t7_badChecksum(); t8_truncStuck();
  std::printf(g_fail ? "GnssCoreTest: %d FAIL\n" : "GnssCoreTest: ALL PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
