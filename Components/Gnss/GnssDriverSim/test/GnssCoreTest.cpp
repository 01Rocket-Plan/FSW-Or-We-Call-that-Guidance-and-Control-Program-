// Edward 2026-11-02 失鎖表：rg100Hz tick；55 s 自 last-good 起算（邊界 15／25／55）。
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
    dev.setLatency(0.0);
    dev.setNoise(0.f, 0.f, 0.f);
    dev.setHdopBias(-0.5f);
    dev.setBaud(115200);
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
  // 單步並在階段變化時記錄 (stage, t, pollsInPrevStage)
  struct Trans { LockStage to; double t; uint32_t pollsPrev; };
  std::vector<Trans> stepWatch(double seconds, LockStage& prev, uint32_t& pollsAtLeave) {
    std::vector<Trans> out;
    const int n = int(seconds / DT + 0.5);
    for (int i = 0; i < n; i++) {
      uint32_t polls = core.pollsInStage();
      t += DT; dev.advance(DT); core.tick(t);
      if (core.lockStage() != prev) {
        out.push_back({core.lockStage(), t, polls});
        pollsAtLeave = polls;
        prev = core.lockStage();
      }
    }
    return out;
  }
};

static void t1_tracking() {
  Rig r; r.run(4.0);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.core.lockStage() == LockStage::TRACKING);
  CHECK(r.sink.fixes.size() >= 3);  // ~1 Hz GGA → 約每秒一筆
  CHECK(r.sink.fixes.back().posValid);
  CHECK(r.sink.fixes.back().fixType == FixType::FIX_3D);
  std::printf("  T1 TRACKING fixes=%zu\n", r.sink.fixes.size());
}

static void t2_velUNaN() {
  Rig r; r.run(2.0);
  CHECK(!r.sink.fixes.empty());
  for (auto& f : r.sink.fixes) { CHECK(std::isnan(f.velEnu_mps[2])); CHECK(!f.velUValid); }
  std::printf("  T2 velU NaN / velUValid=false\n");
}

static void t3_stageLadder() {
  // 短階梯：先 TRACKING（lossNoGga=2），再 NO_DATA。邊界 2.5／3.0／3.7
  // （進入 FAST 約 age=2；其後 0.5／0.5／0.7 跨 MED／SLOW／FAIL）
  Rig r;
  GnssCoreParams p; p.hdopMax = 5.0f;
  p.reacqFastDur_s = 2.50f; p.reacqMedDur_s = 0.50f; p.reacqSlowDur_s = 0.70f;
  p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 30.0f; p.selftestTimeout_s = 2.0f;
  r.core.setParams(p);
  r.run(2.5);
  CHECK(r.core.lockStage() == LockStage::TRACKING);
  const double tGood = r.core.lastGoodPos_s();
  r.dev.setFault(GnssSimFault::NO_DATA);
  for (int i = 0; i < 400 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);
  CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(!r.sink.fixes.empty() && !r.sink.fixes.back().posValid);

  while (r.t - tGood < 2.50) r.run(DT);
  r.run(0.05);
  CHECK(r.core.lockStage() == LockStage::REACQ_MED);
  while (r.t - tGood < 3.00) r.run(DT);
  r.run(0.05);
  CHECK(r.core.lockStage() == LockStage::REACQ_SLOW);
  while (r.t - tGood < 3.70) r.run(DT);
  r.run(0.05);
  CHECK(r.core.lockStage() == LockStage::FIX_FAILED);
  CHECK(r.sink.nFault(GnssFault::FIX_FAILED) == 1);
  CHECK(r.sink.openSky == 1);
  r.run(1.0);
  CHECK(r.sink.nFault(GnssFault::FIX_FAILED) == 1);
  CHECK(r.sink.openSky == 1);
  CHECK(r.core.state() == DrvState::READY);
  std::printf("  T3 ladder -> FIX_FAILED, openSky=1\n");
}

static void t4_recoverEachStage() {
  // FAST
  { Rig r; r.run(2.5); r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 200 && r.core.lockStage() != LockStage::REACQ_FAST; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING); }
  // MED
  { Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.reacqFastDur_s = 2.50f; p.reacqMedDur_s = 10.00f; p.reacqSlowDur_s = 10.00f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 30.0f;
    r.core.setParams(p);
    r.run(2.5); r.dev.setFault(GnssSimFault::NO_DATA);
    for (int i = 0; i < 600 && r.core.lockStage() != LockStage::REACQ_MED; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_MED);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 300 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING); }
  // SLOW
  { Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.reacqFastDur_s = 2.30f; p.reacqMedDur_s = 0.40f; p.reacqSlowDur_s = 10.00f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 30.0f;
    r.core.setParams(p);
    r.run(2.5); r.dev.setFault(GnssSimFault::NO_DATA);
    for (int i = 0; i < 800 && r.core.lockStage() != LockStage::REACQ_SLOW; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_SLOW);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 300 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING); }
  // FIX_FAILED
  { Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.reacqFastDur_s = 2.30f; p.reacqMedDur_s = 0.40f; p.reacqSlowDur_s = 0.50f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 30.0f;
    r.core.setParams(p);
    r.run(2.5); r.dev.setFault(GnssSimFault::NO_DATA);
    for (int i = 0; i < 1000 && r.core.lockStage() != LockStage::FIX_FAILED; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::FIX_FAILED);
    CHECK(r.sink.openSky == 1);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 300 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
    CHECK(r.sink.fixes.back().posValid); }
  std::printf("  T4 recover from FAST/MED/SLOW/FIX_FAILED\n");
}

static void t5_uartSilenceAndStuck() {
  { Rig r; r.run(2.5);
    size_t n = r.sink.fixes.size();
    float age0 = r.sink.fixes.back().dataAge_s;
    r.dev.setFault(GnssSimFault::NO_DATA);
    for (int i = 0; i < 400 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    CHECK(r.core.state() == DrvState::READY);
    CHECK(r.sink.fixes.size() > n);  // 1 Hz 降級
    CHECK(r.sink.fixes.back().dataAge_s >= 1.5f);
    CHECK(r.sink.fixes.back().dataAge_s > age0);
    CHECK(!r.sink.fixes.back().posValid); }
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

static void t6_emitPolicyDataAge() {
  // 新 GGA 才 ++seq；靜默／重鎖期間最多 1 Hz 降級，dataAge 成長
  Rig r;
  GnssCoreParams p; p.hdopMax = 5.0f;
  p.reacqFastDur_s = 30.f; p.reacqMedDur_s = 10.f; p.reacqSlowDur_s = 30.f;
  p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 300.f;
  r.core.setParams(p);
  r.run(3.0);
  CHECK(r.core.lockStage() == LockStage::TRACKING);
  size_t nTrack = r.sink.fixes.size();
  CHECK(nTrack <= 6);
  CHECK(nTrack >= 2);
  uint32_t seqLast = r.sink.fixes.back().seq;
  const double tGood = r.core.lastGoodPos_s();
  r.dev.setFault(GnssSimFault::NO_DATA);
  for (int i = 0; i < 400 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);
  CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
  size_t n0 = r.sink.fixes.size();
  float age0 = r.sink.fixes.back().dataAge_s;
  r.run(3.5);
  size_t added = r.sink.fixes.size() - n0;
  CHECK(added >= 2 && added <= 5);  // ≤1 Hz
  CHECK(!r.sink.fixes.back().posValid);
  CHECK(r.sink.fixes.back().dataAge_s >= age0 + 2.0f);
  CHECK(r.sink.fixes.back().dataAge_s >= float(r.t - tGood) - 1.1f);  // 近即時 age
  for (size_t i = 1; i < r.sink.fixes.size(); i++)
    CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);
  CHECK(r.sink.fixes.back().seq > seqLast);
  std::printf("  T6 emit policy: GGA-only + 1Hz degraded, dataAge grows\n");
}

static void t7_initAcqTimeout() {
  Rig r;
  GnssCoreParams p; p.hdopMax = 5.0f; p.initAcqTimeout_s = 1.5f; p.selftestTimeout_s = 2.0f;
  p.reacqFastDur_s = 5.0f; p.lossNoGga_s = 2.0f;
  r.core.setParams(p);
  r.dev.setFault(GnssSimFault::NO_FIX);
  r.run(1.3);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.core.lockStage() == LockStage::ACQ);
  r.run(1.8);
  // A1：不進 FAULT，進 FIX_FAILED，仍 READY、可恢復
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.core.lockStage() == LockStage::FIX_FAILED);
  CHECK(r.sink.nFault(GnssFault::INIT_ACQ_TIMEOUT) == 1);
  CHECK(r.sink.openSky == 1);
  r.run(1.0);
  CHECK(r.sink.nFault(GnssFault::INIT_ACQ_TIMEOUT) == 1);  // 只一次
  r.dev.setFault(GnssSimFault::NONE);
  for (int i = 0; i < 300 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
  CHECK(r.core.lockStage() == LockStage::TRACKING);
  CHECK(r.core.state() == DrvState::READY);
  std::printf("  T7 ACQ timeout -> FIX_FAILED (not FAULT), then recover\n");
}

static void t8_selftestFail() {
  Rig r;
  GnssCoreParams p; p.selftestTimeout_s = 0.5f; r.core.setParams(p);
  r.dev.setFault(GnssSimFault::NO_DATA);
  r.run(0.8);
  CHECK(r.core.state() == DrvState::FAULT);
  CHECK(r.sink.nFault(GnssFault::SELFTEST_FAIL) >= 1);
  std::printf("  T8 SELFTEST fail -> FAULT\n");
}

static void t9_flapping() {
  Rig r; r.run(2.5);
  for (int c = 0; c < 3; c++) {
    r.dev.setFault(GnssSimFault::NO_FIX);
    for (int i = 0; i < 200 && r.core.lockStage() != LockStage::REACQ_FAST; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 250 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
  }
  CHECK(r.sink.openSky == 0);
  CHECK(r.sink.nLock(LockStage::REACQ_FAST) >= 3);
  CHECK(r.sink.nLock(LockStage::TRACKING) >= 3);
  for (int c = 0; c < 2; c++) {
    r.dev.setFault(GnssSimFault::NO_DATA);
    for (int i = 0; i < 350 && r.core.lockStage() != LockStage::REACQ_FAST; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
    r.dev.setFault(GnssSimFault::NONE);
    for (int i = 0; i < 300 && r.core.lockStage() != LockStage::TRACKING; i++) r.run(DT);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
  }
  std::printf("  T9 flap NO_FIX+NO_DATA, no FIX_FAILED\n");
}

static void t10_hdopAnd2d() {
  Rig r; r.run(2.0);
  r.dev.setFault(GnssSimFault::FIX_2D_ONLY);
  for (int i = 0; i < 200 && r.core.lockStage() == LockStage::TRACKING; i++) r.run(DT);
  CHECK(r.core.lockStage() == LockStage::REACQ_FAST);
  CHECK(!r.sink.fixes.back().posValid);
  r.dev.setFault(GnssSimFault::NONE);
  GnssCoreParams p;
  p.hdopMax = 0.5f;
  p.reacqFastDur_s = 2.50f; p.reacqMedDur_s = 2.00f; p.reacqSlowDur_s = 2.50f;
  p.lossNoGga_s = 2.0f;
  r.core.setParams(p);
  for (int i = 0; i < 300; i++) r.run(DT);
  std::printf("  T10 2D->REACQ, hdop gate exercised\n");
}

static void t11_seqContiguous() {
  Rig r; r.run(2.5);
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(3.0);  // 含 1 Hz 降級
  r.dev.setFault(GnssSimFault::NONE); r.run(2.0);
  CHECK(r.sink.fixes.size() >= 2);
  for (size_t i = 1; i < r.sink.fixes.size(); i++)
    CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);
  std::printf("  T11 seq contiguous across silence\n");
}

// T12：生產時長 15／10／30；中斷 12／20／60 s；檢查進入時刻與各階段 poll 數
static void t12_stageTimingPolls() {
  auto runOutage = [](double outage_s, LockStage expectFinal,
                      double* enterFast, double* enterMed, double* enterSlow, double* enterFail,
                      uint32_t* pollsFast, uint32_t* pollsMed, uint32_t* pollsSlow) {
    Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.reacqFastDur_s = 15.f; p.reacqMedDur_s = 10.f; p.reacqSlowDur_s = 30.f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 300.f;
    p.pollTracking_s = 0.1f; p.pollReacqFast_s = 0.1f;
    p.pollReacqMed_s = 1.0f; p.pollReacqSlow_s = 2.0f;
    r.core.setParams(p);
    r.run(3.0);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
    const double tGood = r.core.lastGoodPos_s();
    r.dev.setFault(GnssSimFault::NO_DATA);

    LockStage prev = LockStage::TRACKING;
    uint32_t pollsLeave = 0;
    *enterFast = *enterMed = *enterSlow = *enterFail = -1;
    *pollsFast = *pollsMed = *pollsSlow = 0;

    // 跑到 last-good + outage_s
    while (r.t < tGood + outage_s - 1e-9) {
      uint32_t polls = r.core.pollsInStage();
      r.run(DT);
      if (r.core.lockStage() != prev) {
        if (r.core.lockStage() == LockStage::REACQ_FAST) {
          *enterFast = r.t - tGood;
        } else if (r.core.lockStage() == LockStage::REACQ_MED) {
          *enterMed = r.t - tGood;
          *pollsFast = polls;  // polls in FAST just before leave (captured before tick... careful)
        } else if (r.core.lockStage() == LockStage::REACQ_SLOW) {
          *enterSlow = r.t - tGood;
          *pollsMed = polls;
        } else if (r.core.lockStage() == LockStage::FIX_FAILED) {
          *enterFail = r.t - tGood;
          *pollsSlow = polls;
        }
        // 修正：polls 應為轉換前的值。step 後 pollsInStage 已歸零。
        // 上面 polls 取的是 tick 前，正確。
        prev = r.core.lockStage();
        (void)pollsLeave;
      }
    }
    // 若仍在某階段，記錄當下 polls
    if (r.core.lockStage() == LockStage::REACQ_FAST) *pollsFast = r.core.pollsInStage();
    if (r.core.lockStage() == LockStage::REACQ_MED) *pollsMed = r.core.pollsInStage();
    if (r.core.lockStage() == LockStage::REACQ_SLOW) *pollsSlow = r.core.pollsInStage();
    CHECK(r.core.lockStage() == expectFinal);
    return tGood;
  };

  // --- 12 s：應仍在 FAST；進入 ≈ lossNoGga(2)；poll ≈ (12-2)/0.1 = 100 ---
  {
    double eF, eM, eS, eX; uint32_t pF, pM, pS;
    runOutage(12.0, LockStage::REACQ_FAST, &eF, &eM, &eS, &eX, &pF, &pM, &pS);
    CHECK(eF > 1.5 && eF < 2.5);       // ~ LOSS_NO_GGA_S
    CHECK(eM < 0);                     // 未進 MED
    CHECK(pF >= 90 && pF <= 110);      // ~100
    std::printf("  T12a outage=12s enterFAST=%.2f pollsFAST=%u\n", eF, pF);
  }
  // --- 20 s：MED@15；FAST polls~(15-2)/0.1=130；MED polls~(20-15)/1=5 ---
  {
    double eF, eM, eS, eX; uint32_t pF, pM, pS;
    runOutage(20.0, LockStage::REACQ_MED, &eF, &eM, &eS, &eX, &pF, &pM, &pS);
    CHECK(eF > 1.5 && eF < 2.5);
    CHECK(eM > 14.5 && eM < 15.5);
    CHECK(eS < 0);
    CHECK(pF >= 110 && pF <= 140);
    CHECK(pM >= 3 && pM <= 7);
    std::printf("  T12b outage=20s enterMED=%.2f pollsFAST=%u pollsMED=%u\n", eM, pF, pM);
  }
  // --- 60 s：SLOW@25、FAIL@55；SLOW polls~(55-25)/2=15 ---
  {
    double eF, eM, eS, eX; uint32_t pF, pM, pS;
    runOutage(60.0, LockStage::FIX_FAILED, &eF, &eM, &eS, &eX, &pF, &pM, &pS);
    CHECK(eF > 1.5 && eF < 2.5);
    CHECK(eM > 14.5 && eM < 15.5);
    CHECK(eS > 24.5 && eS < 25.5);
    CHECK(eX > 54.5 && eX < 55.5);
    CHECK(pF >= 110 && pF <= 140);
    CHECK(pM >= 8 && pM <= 12);        // (25-15)/1 = 10
    CHECK(pS >= 13 && pS <= 17);       // (55-25)/2 = 15
    std::printf("  T12c outage=60s enterSLOW=%.2f enterFAIL=%.2f pollsF/M/S=%u/%u/%u\n",
                eS, eX, pF, pM, pS);
  }
  std::printf("  T12 stage timing + poll counts (12/20/60s) OK\n");
}

int main() {
  t1_tracking(); t2_velUNaN(); t3_stageLadder(); t4_recoverEachStage();
  t5_uartSilenceAndStuck(); t6_emitPolicyDataAge(); t7_initAcqTimeout();
  t8_selftestFail(); t9_flapping(); t10_hdopAnd2d(); t11_seqContiguous();
  t12_stageTimingPolls();
  std::printf(g_fail ? "GnssCoreTest: %d FAIL\n" : "GnssCoreTest: ALL PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
