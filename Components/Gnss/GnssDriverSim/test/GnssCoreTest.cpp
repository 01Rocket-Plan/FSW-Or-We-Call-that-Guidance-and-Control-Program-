// T1~T8 + T10~T13：真 GnssCore＋ GnssSimDevice，tick = rg1Hz（1 s）
// Gate C6：READY 後失鎖／UART 靜默不進 FAULT（預設 NODATA_FAULT_TICKS=0）；
//          posValid=false、dataAge↑、FIX_LOST／NO_DATA edge；恢復 FIX_RECOVERED。
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
  // C6：開機後全程 NO_DATA → SELFTEST 失敗進 FAULT（尚未 READY）
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
  CHECK(hi > 0);  // bias 讓超門檻可直接測到
  GnssCoreParams p; p.hdopMax = 0.5f; r.core.setParams(p); r.dev.setHdopBias(0.f); size_t n0 = r.sink.fixes.size(); r.run(5);
  for (size_t i = n0; i < r.sink.fixes.size(); i++) {
    if (r.sink.fixes[i].fixType == FixType::FIX_3D) CHECK(!r.sink.fixes[i].posValid);
  }
}
static void t5_seq() {
  // C6 預設：READY 後 NO_DATA 不進 FAULT；seq 連續；可選 NODATA_FAULT_TICKS>0 才進 FAULT
  Rig r; r.run(10); uint32_t s = r.sink.fixes.back().seq; size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(8);  // 預設永不 FAULT
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() == n + 8);
  CHECK(r.sink.fixes.back().seq == s + 8);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) == 1);  // edge 只報一次
  r.dev.setFault(GnssSimFault::NONE); r.run(10);
  CHECK(r.core.state() == DrvState::READY);
  for (size_t i = 1; i < r.sink.fixes.size(); i++) CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);

  // 可選：設定 noDataFaultTicks=5 → 仍可進 FAULT（地面 BIT）
  Rig q; GnssCoreParams p; p.noDataFaultTicks = 5; p.noDataWarnTicks = 2; q.core.setParams(p);
  q.run(10); size_t nq = q.sink.fixes.size();
  q.dev.setFault(GnssSimFault::NO_DATA); q.run(8);
  CHECK(q.core.state() == DrvState::FAULT);
  size_t nFault = q.sink.fixes.size();
  CHECK(nFault >= nq);  // FAULT 前仍有輸出
  q.run(3); CHECK(q.sink.fixes.size() == nFault);  // FAULT 中不輸出
  q.dev.setFault(GnssSimFault::NONE); q.run(15);
  CHECK(q.core.state() == DrvState::READY && q.sink.fixes.size() > nFault);
}
static void t6_dataAge() {
  // NO_DATA 期間（仍 READY）dataAge_s 單調遞增；逾 FIX_STALE_AGE_S 後 posValid=false
  Rig r; r.run(10); size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(3);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.fixes.size() >= n + 3);
  float a0 = r.sink.fixes[n].dataAge_s, a1 = r.sink.fixes[n + 1].dataAge_s, a2 = r.sink.fixes[n + 2].dataAge_s;
  std::printf("  dataAge during NO_DATA: %.2f %.2f %.2f\n", a0, a1, a2);
  CHECK(a1 > a0 && a2 > a1);
  CHECK(a2 >= 2.0f);
  CHECK(!r.sink.fixes.back().posValid);  // stale
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
  // C6：READY 後 NO_DATA 預設不進 FAULT
  { Rig r; r.run(10); r.dev.setFault(GnssSimFault::NO_DATA); r.run(8);
    CHECK(r.core.state() == DrvState::READY && r.sink.nFault(GnssFault::NO_DATA) == 1);
    r.dev.setFault(GnssSimFault::NONE); r.run(10); CHECK(r.core.state() == DrvState::READY); }
}

// T10：飛行中失鎖（仍收 NMEA，品質 0）→ 維持 READY、posValid=false、FIX_LOST 一次
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
  // Sim 有 latency：注入後前 1~2 個 epoch 可能仍是舊 3D；只檢查最後 3 筆已切到 NO_FIX
  for (size_t i = r.sink.fixes.size() - 3; i < r.sink.fixes.size(); i++) {
    CHECK(!r.sink.fixes[i].posValid);
    CHECK(r.sink.fixes[i].fixType == FixType::NO_FIX);
  }
  std::printf("  T10 FIX_LOST once, still READY, posValid=false\n");
}

// T11：飛行中 UART 靜默 → READY、dataAge↑、NO_DATA 一次、逾齡 posValid=false
static void t11_uartSilenceMidFlight() {
  Rig r; r.run(12); size_t n = r.sink.fixes.size();
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(5);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) == 1);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == 1);  // stale → FIX_LOST
  CHECK(r.sink.fixes.size() == n + 5);
  CHECK(r.sink.fixes.back().dataAge_s >= 4.0f);
  CHECK(!r.sink.fixes.back().posValid);
  std::printf("  T11 UART silence: NO_DATA+FIX_LOST edge, dataAge=%.2f\n", r.sink.fixes.back().dataAge_s);
}

// T12：失鎖／靜默後恢復 → FIX_RECOVERED、posValid 回來
static void t12_recovery() {
  Rig r; r.run(10);
  r.dev.setFault(GnssSimFault::NO_FIX); r.run(4);
  CHECK(r.sink.nFault(GnssFault::FIX_LOST) == 1);
  r.dev.setFault(GnssSimFault::NONE); r.run(8);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) == 1);
  CHECK(!r.core.fixLostLatched());
  CHECK(r.sink.fixes.back().posValid);
  // 再測 UART 靜默恢復
  size_t rec0 = r.sink.nFault(GnssFault::FIX_RECOVERED);
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(4);
  CHECK(r.sink.nFault(GnssFault::NO_DATA) >= 1);
  r.dev.setFault(GnssSimFault::NONE); r.run(8);
  CHECK(r.sink.nFault(GnssFault::FIX_RECOVERED) == rec0 + 1);
  CHECK(r.sink.fixes.back().posValid);
  std::printf("  T12 recovery OK\n");
}

// T13：失鎖拍動（NO_FIX ↔ 正常）→ 每一段失鎖只報一次 FIX_LOST，恢復報 FIX_RECOVERED
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
  std::printf("  T13 flapping: FIX_LOST=%zu FIX_RECOVERED=%zu\n",
              r.sink.nFault(GnssFault::FIX_LOST), r.sink.nFault(GnssFault::FIX_RECOVERED));
}

int main() {
  t1_readyValid(); t2_velUNaN(); t3_fix2d(); t4_hdop(); t5_seq(); t6_dataAge(); t7_badChecksum(); t8_truncStuck();
  t10_lockLostMidFlight(); t11_uartSilenceMidFlight(); t12_recovery(); t13_flapping();
  std::printf(g_fail ? "GnssCoreTest: %d FAIL\n" : "GnssCoreTest: ALL PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
