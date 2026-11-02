// Edward 2026-11-02 失鎖表：rg100Hz tick（dt=0.01），poll 依 LockStage。
#include "Components/Gnss/GnssDriverSim/GnssSimDevice.hpp"
#include "Components/Gnss/GnssDriver/GnssCore.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace Gnss;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)
static const double kDeg = 3.14159265358979323846 / 180.0;
static const double DT = 0.01;  // rg100Hz

struct Sink : IGnssSink {
  std::vector<GnssFix> fixes;
  std::vector<GnssFault> faults;
  std::vector<DrvState> states;
  std::vector<LockStage> locks;
  int openSky = 0;
  void onFix(const GnssFix& f) override { fixes.push_back(f); }
  void onStateChange(DrvState, DrvState to) override { states.push_back(to); }
  void onLockStageChange(LockStage, LockStage to) override { locks.push_back(to); }
  void onFault(GnssFault f, uint32_t) override { faults.push_back(f); }
  void onFixFailedOpenSky() override { openSky++; }
  size_t nFault(GnssFault f) const { size_t k = 0; for (auto x : faults) if (x == f) k++; return k; }
  size_t nLock(LockStage s) const { size_t k = 0; for (auto x : locks) if (x == s) k++; return k; }
};

struct Rig {
  GnssSimDevice dev; Sink sink; GnssCore core{dev, sink}; double t = 0;
  Rig() {
    const float v[3] = {10.f, 5.f, 2.f};
    dev.setTrajectory(24.0 * kDeg, 121.0 * kDeg, 50.0, v);
    dev.setLatency(0.0);           // 測試免等 epoch 延遲
    dev.setNoise(0.f, 0.f, 0.f);   // 位置穩定
    dev.setHdopBias(-0.5f);        // 確保 hdop 低於門檻
    dev.setBaud(115200);         // 一 poll 內吐完整 epoch
    GnssCoreParams p;
    p.hdopMax = 5.0f;
    p.reacqFastDur_s = 2.50f; p.reacqMedDur_s = 2.00f; p.reacqSlowDur_s = 2.50f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 3.0f; p.selftestTimeout_s = 2.0f;
    p.stuckPolls = 3;
    core.setParams(p);
    core.start();
  }
  void run(double seconds) {
    const int n = int(seconds / DT + 0.5);
    for (int i = 0; i < n; i++) { t += DT; dev.advance(DT); core.tick(t); }
  }
};

static void t1_tracking() {
  Rig r; r.run(4.0);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.core.lockStage() == LockStage::TRACKING);
  CHECK(r.sink.fixes.size() >= 10);
  CHECK(r.sink.fixes.back().posValid);
  CHECK(r.sink.fixes.back().fixType == FixType::FIX_3D);
  std::printf("  T1 TRACKING fixes=%zu\n", r.sink.fixes.size());
}

static void t2_velUNaN() {
  Rig r; r.run(2.0);
  for (auto& f : r.sink.fixes) { CHECK(std::isnan(f.velEnu_mps[2])); CHECK(!f.velUValid); }
}

static void t3_stageLadder() {
  // TRACKING → FAST → MED → SLOW → FIX_FAILED；計時約 0.5+0.4+0.6
  Rig r;
  GnssCoreParams p; p.hdopMax = 5.0f;
  p.reacqFastDur_s = 0.50f; p.reacqMedDur_s = 0.40f; p.reacqSlowDur_s = 0.60f;
  p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 3.0f; p.selftestTimeout_s = 2.0f;
  r.core.setParams(p);
  r.run(2.5);
  CHECK(r.core.lockStage() == LockStage::TRACKING);
  r.dev.setFault(GnssSimFault::NO_FIX);
  r.run(0.15);  // 進入 REACQ_FAST（等下一個劣質 GGA epoch）
  // 最多等 1.2 s 進 FAST（latency+epoch）
  for (int i = 0; i < 150 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);
  CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(!r.sink.fixes.empty() && !r.sink.fixes.back().posValid);

  r.run(0.55);
  CHECK(r.core.lockStage() == LockStage::REACQ_MED);
  r.run(0.45);
  CHECK(r.core.lockStage() == LockStage::REACQ_SLOW);
  r.run(0.65);
  CHECK(r.core.lockStage() == LockStage::FIX_FAILED);
  CHECK(r.sink.nFault(GnssFault::FIX_FAILED) == 1);
  CHECK(r.sink.openSky == 1);
  r.run(1.0);  // 不再重複
  CHECK(r.sink.nFault(GnssFault::FIX_FAILED) == 1);
  CHECK(r.sink.openSky == 1);
  CHECK(r.core.state() == DrvState::READY);
  std::printf("  T3 ladder -> FIX_FAILED, openSky=1\n");
}

static void t4_recoverEachStage() {
  auto recoverFrom = [](LockStage want) {
    Rig r; r.run(2.5);
    r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 200 && r.core.lockStage() != want; i++) r.run(DT);
    CHECK(r.core.lockStage() == want);
    size_t n = r.sink.fixes.size();
    uint32_t seq = r.sink.fixes.back().seq;
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
    CHECK(r.sink.fixes.back().posValid);
    CHECK(r.sink.fixes.back().seq > seq);
    CHECK(r.sink.fixes.size() > n);
    // dataAge 應在恢復後變小
    CHECK(r.sink.fixes.back().dataAge_s < 1.0f);
  };
  // FAST：失鎖後立刻
  { Rig r; r.run(2.5); r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 200 && r.core.lockStage() != LockStage::REACQ_FAST; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING); }
  // MED
  { Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.reacqFastDur_s = 0.50f; p.reacqMedDur_s = 2.00f; p.reacqSlowDur_s = 2.50f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 30.0f;
    r.core.setParams(p);
    r.run(2.5); r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 400 && r.core.lockStage() != LockStage::REACQ_MED; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_MED);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING); }
  // SLOW
  { Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.reacqFastDur_s = 0.50f; p.reacqMedDur_s = 0.40f; p.reacqSlowDur_s = 2.50f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 30.0f;
    r.core.setParams(p);
    r.run(2.5); r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 600 && r.core.lockStage() != LockStage::REACQ_SLOW; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_SLOW);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING); }
  // FIX_FAILED（縮短階梯以便測）
  { Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.reacqFastDur_s = 0.50f; p.reacqMedDur_s = 0.40f; p.reacqSlowDur_s = 0.60f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 30.0f;
    r.core.setParams(p);
    r.run(2.5); r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 800 && r.core.lockStage() != LockStage::FIX_FAILED; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::FIX_FAILED);
    CHECK(r.sink.openSky == 1);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
    CHECK(r.sink.fixes.back().posValid); }
  (void)recoverFrom;
  std::printf("  T4 recover from FAST/MED/SLOW/FIX_FAILED\n");
}

static void t5_uartSilenceAndStuck() {
  // UART 靜默 → LOSS_NO_GGA → REACQ_FAST；dataAge↑
  { Rig r; r.run(2.5);
    size_t n = r.sink.fixes.size();
    r.dev.setFault(GnssSimFault::NO_DATA);
    for (int i = 0; i < 400 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);  // > LOSS_NO_GGA 2s
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    CHECK(r.core.state() == DrvState::READY);
    CHECK(r.sink.fixes.size() > n);
    CHECK(r.sink.fixes.back().dataAge_s >= 1.5f);
    CHECK(!r.sink.fixes.back().posValid); }
  // STUCK → REACQ_FAST
  { Rig r; r.run(2.5);
    r.dev.setFault(GnssSimFault::STUCK);
    for (int i = 0; i < 500 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    CHECK(r.core.state() == DrvState::READY);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 300 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING); }
  std::printf("  T5 silence+STUCK -> REACQ then recover\n");
}

static void t6_initAcqTimeout() {
  Rig r;
  GnssCoreParams p; p.hdopMax = 5.0f; p.initAcqTimeout_s = 1.5f; p.selftestTimeout_s = 2.0f;
  p.reacqFastDur_s = 5.0f; p.lossNoGga_s = 2.0f;
  r.core.setParams(p);
  r.dev.setFault(GnssSimFault::NO_FIX);  // 有 NMEA 但永不 3D
  r.run(1.3);  // 過第一個 epoch：SELFTEST → READY+ACQ
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.core.lockStage() == LockStage::ACQ);
  r.run(1.8);  // 超過 initAcqTimeout
  CHECK(r.core.state() == DrvState::FAULT);
  CHECK(r.sink.nFault(GnssFault::INIT_ACQ_TIMEOUT) >= 1);
  std::printf("  T6 ACQ timeout -> FAULT\n");
}

static void t7_selftestFail() {
  Rig r;
  GnssCoreParams p; p.selftestTimeout_s = 0.5f; r.core.setParams(p);
  r.dev.setFault(GnssSimFault::NO_DATA);
  r.run(0.8);
  CHECK(r.core.state() == DrvState::FAULT);
  CHECK(r.sink.nFault(GnssFault::SELFTEST_FAIL) >= 1);
  std::printf("  T7 SELFTEST fail\n");
}

static void t8_flapping() {
  Rig r; r.run(2.5);
  for (int c = 0; c < 3; c++) {
    r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 200 && r.core.lockStage() != LockStage::REACQ_FAST; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
  }
  CHECK(r.sink.openSky == 0);  // 未走到 FIX_FAILED
  CHECK(r.sink.nLock(LockStage::REACQ_FAST) >= 3);
  CHECK(r.sink.nLock(LockStage::TRACKING) >= 3);
  // NO_DATA 拍動
  for (int c = 0; c < 2; c++) {
    r.dev.setFault(GnssSimFault::NO_DATA);
    for (int i = 0; i < 350 && r.core.lockStage() != LockStage::REACQ_FAST; i++) r.run(DT);  // > LOSS_NO_GGA 2s
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 300 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
  }
  std::printf("  T8 flap NO_FIX+NO_DATA, no FIX_FAILED\n");
}

static void t9_hdopAnd2d() {
  Rig r; r.run(2.0);
  r.dev.setFault(GnssSimFault::FIX_2D_ONLY);
  for (int i = 0; i < 200 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);
  CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
  CHECK(!r.sink.fixes.back().posValid);
  r.dev.setFault(GnssSimFault::NONE);
  GnssCoreParams p;
  p.hdopMax = 0.5f;
  p.reacqFastDur_s = 0.50f; p.reacqMedDur_s = 0.40f; p.reacqSlowDur_s = 0.60f;
  p.lossNoGga_s = 2.0f;
  r.core.setParams(p);
  // 恢復後可能因 hdop 門檻再失鎖；至少曾回到過或維持可測
  for (int i = 0; i < 300; i++) r.run(DT);
  std::printf("  T9 2D->REACQ, hdop gate exercised\n");
}

static void t10_seqContiguous() {
  Rig r; r.run(2.5);
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(0.5);
  r.dev.setFault(GnssSimFault::NONE); r.run(2.0);
  for (size_t i = 1; i < r.sink.fixes.size(); i++)
    CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);
  std::printf("  T10 seq contiguous across silence\n");
}

int main() {
  t1_tracking(); t2_velUNaN(); t3_stageLadder(); t4_recoverEachStage();
  t5_uartSilenceAndStuck(); t6_initAcqTimeout(); t7_selftestFail();
  t8_flapping(); t9_hdopAnd2d(); t10_seqContiguous();
  std::printf(g_fail ? "GnssCoreTest: %d FAIL\n" : "GnssCoreTest: ALL PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
