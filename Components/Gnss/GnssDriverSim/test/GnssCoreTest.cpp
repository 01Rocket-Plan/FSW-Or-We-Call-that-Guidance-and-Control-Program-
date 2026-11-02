// T1~T8 + T10~T15：真 GnssCore＋ GnssSimDevice，tick = rg1Hz（1 s）
// C6：READY 後失鎖／靜默／STUCK 不進 FAULT；E4：NODATA_TIMEOUT edge。
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
  for (auto& f : r.sink.fixes) CHECK(f.valid);
  Rig q; q.dev.setFault(GnssSimFault::NO_DATA); q.run(10);
  CHECK(q.core.state() == DrvState::FAULT && q.sink.fixes.empty());
  CHECK(q.sink.nFault(GnssFault::SELFTEST_FAIL) >= 1);
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
  CHECK(hi > 0);
  GnssCoreParams p; p.hdopMax = 0.5f; r.core.setParams(p); r.dev.setHdopBias(0.f); size_t n0 = r.sink.fixes.size(); r.run(5);
  for (size_t i = n0; i < r.sink.fixes.size(); i++) {
    if (r.sink.fixes[i].fixType == FixType::FIX_3D) CHECK(!r.sink.fixes[i].posValid);
  }
}
static void t5_seq() {
  Rig r; r.run(10); uint32_t s = r.sink.fixes.back().seq; size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(8);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() == n + 8);
  CHECK(r.sink.fixes.back().seq == s + 8);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) == 1);
  r.dev.setFault(GnssSimFault::NONE); r.run(10);
  CHECK(r.core.state() == DrvState::READY);
  for (size_t i = 1; i < r.sink.fixes.size(); i++) CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);

  Rig q; GnssCoreParams p; p.noDataFaultTicks = 5; p.noDataWarnTicks = 2; p.noDataTimeoutTicks = 100; q.core.setParams(p);
  q.run(10); size_t nq = q.sink.fixes.size();
  q.dev.setFault(GnssSimFault::NO_DATA); q.run(8);
  CHECK(q.core.state() == DrvState::FAULT);
  size_t nFault = q.sink.fixes.size();
  CHECK(nFault >= nq);
  q.run(3); CHECK(q.sink.fixes.size() == nFault);
  q.dev.setFault(GnssSimFault::NONE); q.run(15);
  CHECK(q.core.state() == DrvState::READY && q.sink.fixes.size() > nFault);
}
static void t6_dataAge() {
  Rig r; r.run(10); size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(3);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() >= n + 3);
  float a0 = r.sink.fixes[n].dataAge_s, a1 = r.sink.fixes[n + 1].dataAge_s, a2 = r.sink.fixes[n + 2].dataAge_s;
  std::printf("  dataAge during NO_DATA: %.2f %.2f %.2f\n", a0, a1, a2);
  CHECK(a1 > a0 && a2 > a1);
  CHECK(a2 >= 2.0f);
  CHECK(!r.sink.fixes.back().posValid);
}
static void t7_badChecksum() {
  Rig r; r.run(10); r.dev.setFault(GnssSimFault::BAD_CHECKSUM); r.run(5);
  CHECK(r.core.badChecksumCount() >= 4 && r.sink.nFault(GnssFault::BAD_CHECKSUM) >= 4);
  r.dev.setFault(GnssSimFault::NONE); size_t n = r.sink.fixes.size(); r.run(10);
  CHECK(r.core.state() == DrvState::READY && r.sink.fixes.size() > n);
}
static void t8_truncStuck() {
  { Rig r; r.run(10); r.dev.setFault(GnssSimFault::TRUNCATED); r.run(5);
    std::printf("  trunc count %u, TRUNCATED faults %zu, state %d\n",
                r.core.truncatedCount(), r.sink.nFault(GnssFault::TRUNCATED), (int)r.core.state());
    CHECK(r.core.truncatedCount() >= 4 && r.sink.nFault(GnssFault::TRUNCATED) >= 4);
    CHECK(r.core.state() == DrvState::READY);
    r.dev.setFault(GnssSimFault::NONE); r.run(10); CHECK(r.core.state() == DrvState::READY); }
  // READY 後 STUCK：維持 READY（降級），不再進 FAULT
  { Rig r; r.run(10); r.dev.setFault(GnssSimFault::STUCK); r.run(8);
    CHECK(r.core.state() == DrvState::READY && r.sink.nFault(GnssFault::STUCK) == 1);
    r.dev.setFault(GnssSimFault::NONE); r.run(10); CHECK(r.core.state() == DrvState::READY); }
  { Rig r; r.run(10); r.dev.setFault(GnssSimFault::NO_DATA); r.run(8);
    CHECK(r.core.state() == DrvState::READY && r.sink.nFault(GnssFault::NO_DATA) == 1);
    r.dev.setFault(GnssSimFault::NONE); r.run(10); CHECK(r.core.state() == DrvState::READY); }
}

static void t10_lockLostMidFlight() {
  Rig r; r.run(12);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.back().posValid);
  size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_FIX); r.run(6);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() == n + 6);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == 1);
  CHECK(r.core.fixLostLatched());
  for (size_t i = r.sink.fixes.size() - 3; i < r.sink.fixes.size(); i++) {
    CHECK(!r.sink.fixes[i].posValid);
    CHECK(r.sink.fixes[i].fixType == FixType::NO_FIX);
  }
  std::printf("  T10 FIX_LOST once, still READY, posValid=false\n");
}

static void t11_uartSilenceMidFlight() {
  Rig r; r.run(12); size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(5);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) == 1);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == 1);
  CHECK(r.sink.fixes.size() == n + 5);
  CHECK(r.sink.fixes.back().dataAge_s >= 4.0f);
  CHECK(!r.sink.fixes.back().posValid);
  std::printf("  T11 UART silence: NO_DATA+FIX_LOST edge, dataAge=%.2f\n", r.sink.fixes.back().dataAge_s);
}

static void t12_recovery() {
  Rig r; r.run(10);
  r.dev.setFault(GnssSimFault::NO_FIX); r.run(4);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == 1);
  r.dev.setFault(GnssSimFault::NONE); r.run(8);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) == 1);
  CHECK(!r.core.fixLostLatched());
  CHECK(r.sink.fixes.back().posValid);
  size_t rec0 = r.sink.nFault(GnssFault::FIX_RECOVERED);
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(4);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) >= 1);
  r.dev.setFault(GnssSimFault::NONE); r.run(8);
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) == rec0 + 1);
  CHECK(r.sink.fixes.back().posValid);
  std::printf("  T12 recovery OK\n");
}

// T13：NO_FIX 拍動 + NO_DATA 拍動 → 每段 episode 各一次 edge
static void t13_flapping() {
  Rig r; r.run(10);
  for (int cycle = 0; cycle < 3; cycle++) {
    r.dev.setFault(GnssSimFault::NO_FIX); r.run(3);
    r.dev.setFault(GnssSimFault::NONE); r.run(4);
  }
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == 3);
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) == 3);
  CHECK(r.sink.fixes.back().posValid);

  size_t noData0 = r.sink.nFault(GnssFault::NO_DATA);
  size_t lost0 = r.sink.nFault(GnssFault::FIX_LOST);
  size_t rec0 = r.sink.nFault(GnssFault::FIX_RECOVERED);
  for (int cycle = 0; cycle < 3; cycle++) {
    r.dev.setFault(GnssSimFault::NO_DATA); r.run(4);  // warn at 2；未達預設 timeout 10
    r.dev.setFault(GnssSimFault::NONE); r.run(5);
  }
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) == noData0 + 3);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == lost0 + 3);
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) == rec0 + 3);
  CHECK(r.sink.fixes.back().posValid);
  std::printf("  T13 flap NO_FIX+NO_DATA: NO_DATA=%zu FIX_LOST=%zu FIX_RECOVERED=%zu\n",
              r.sink.nFault(GnssFault::NO_DATA), r.sink.nFault(GnssFault::FIX_LOST),
              r.sink.nFault(GnssFault::FIX_RECOVERED));
}

// T14：E4 完全逾時 → NODATA_TIMEOUT 一次／episode，維持 READY
static void t14_nodataTimeout() {
  Rig r; GnssCoreParams p; p.noDataWarnTicks = 2; p.noDataTimeoutTicks = 5; p.noDataFaultTicks = 0;
  r.core.setParams(p);
  r.run(10);
  CHECK(r.core.state() == DrvState::READY);
  size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(8);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() == n + 8);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) == 1);
  CHECK(r.sink.nFault(GnssFault::NODATA_TIMEOUT) == 1);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == 1);
  CHECK(!r.sink.fixes.back().posValid);
  // 恢復後再靜默 → 再各報一次
  r.dev.setFault(GnssSimFault::NONE); r.run(8);
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) >= 1);
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(8);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) == 2);
  CHECK(r.sink.nFault(GnssFault::NODATA_TIMEOUT) == 2);
  std::printf("  T14 NODATA_TIMEOUT x2, still READY\n");
}

// T15：READY 後 STUCK → 降級（posValid false、dataAge↑、STUCK 一次），恢復後 FIX_RECOVERED、seq 連續
static void t15_stuckRecovery() {
  Rig r; r.run(12);
  CHECK(r.core.state() == DrvState::READY && r.sink.fixes.back().posValid);
  uint32_t seq0 = r.sink.fixes.back().seq;
  size_t n0 = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::STUCK); r.run(8);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::STUCK) == 1);
  CHECK(r.core.stuckLatched());
  CHECK(r.sink.fixes.size() == n0 + 8);
  CHECK(r.sink.fixes.back().seq == seq0 + 8);
  // 最後幾筆：posValid false、dataAge 增長
  float aPrev = -1.f;
  for (size_t i = r.sink.fixes.size() - 4; i < r.sink.fixes.size(); i++) {
    CHECK(!r.sink.fixes[i].posValid);
    CHECK(r.sink.fixes[i].dataAge_s > aPrev);
    aPrev = r.sink.fixes[i].dataAge_s;
  }
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) >= 1);
  r.dev.setFault(GnssSimFault::NONE); r.run(10);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(!r.core.stuckLatched());
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) >= 1);
  CHECK(r.sink.fixes.back().posValid);
  for (size_t i = 1; i < r.sink.fixes.size(); i++)
    CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);
  std::printf("  T15 STUCK degrade+recover, dataAge grew, seq contiguous\n");
}

// T16：SELFTEST 中 STUCK → FAULT（帶起仍嚴格）
static void t16_stuckSelftestFault() {
  Rig r;
  GnssCoreParams p; p.selftestTicks = 20; p.stuckTicks = 3; r.core.setParams(p);
  r.dev.setFault(GnssSimFault::STUCK);
  r.run(12);
  CHECK(r.core.state() == DrvState::FAULT);
  CHECK(r.sink.nFault(GnssFault::STUCK) >= 1);
  CHECK(r.sink.fixes.empty());
  std::printf("  T16 SELFTEST STUCK -> FAULT\n");
}

int main() {
  t1_readyValid(); t2_velUNaN(); t3_fix2d(); t4_hdop(); t5_seq(); t6_dataAge(); t7_badChecksum(); t8_truncStuck();
  t10_lockLostMidFlight(); t11_uartSilenceMidFlight(); t12_recovery(); t13_flapping();
  t14_nodataTimeout(); t15_stuckRecovery(); t16_stuckSelftestFault();
  std::printf(g_fail ? "GnssCoreTest: %d FAIL\n" : "GnssCoreTest: ALL PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
