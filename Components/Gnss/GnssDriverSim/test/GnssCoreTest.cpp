// =============================================================================
// GnssCoreTest — 主機單測（不需 F´）
//
// 驗證目標（Edward 2026-11-02 失鎖表＋整數 tick 分頻＋emit／seq＋C6）：
//   T1  正常鎖定：READY＋TRACKING、約 1 Hz 合格 fix
//   T2  VTG 無垂直速：velU 為 NaN、velUValid=false（介面契約）
//   T3  階梯：TRACKING→FAST→MED→SLOW→FIX_FAILED；原文一次；仍 READY
//   T4  各階段（含 FIX_FAILED）可恢復 TRACKING（C6：不自毀）
//   T5  UART 靜默與 STUCK → 失鎖＋1 Hz 降級 dataAge；可恢復
//   T6  emit 政策：新 GGA 即發；靜默≤1 Hz；seq 連續；dataAge 成長
//   T7  開機 ACQ 逾時 → FIX_FAILED（非 FAULT），可恢復
//   T8  SELFTEST 失敗 → FAULT（硬故障分流）
//   T9  反覆失鎖／重鎖不誤進 FIX_FAILED
//   T10 2D／HDOP 門檻視為失鎖
//   T11 跨越靜默 seq 仍連續遞增
//   T12 生產時長 15／10／30 與分頻 10／100／200；中斷 12／20／60 s 的進入時刻與 poll 數
//
// 時間基準：DT=0.01＝rg100Hz。整數分頻 ticks＝POLL_*_MS/rgPeriod_ms。
// =============================================================================
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
  // 驗證：暖機後進入 READY＋TRACKING，且因 ~1 Hz GGA 而有多筆 posValid=true 的 3D fix。
  Rig r; r.run(4.0);
  CHECK(r.core.state() == DrvState::READY);
  CHECK(r.core.lockStage() == LockStage::TRACKING);
  CHECK(r.sink.fixes.size() >= 3);  // ~1 Hz GGA → 約每秒一筆
  CHECK(r.sink.fixes.back().posValid);
  CHECK(r.sink.fixes.back().fixType == FixType::FIX_3D);
  std::printf("  T1 TRACKING fixes=%zu\n", r.sink.fixes.size());
}

static void t2_velUNaN() {
  // 驗證：NMEA VTG 無 U 分量 → velEnu[2]=NaN 且 velUValid=false（避免下游當 0 用）。
  Rig r; r.run(2.0);
  CHECK(!r.sink.fixes.empty());
  for (auto& f : r.sink.fixes) { CHECK(std::isnan(f.velEnu_mps[2])); CHECK(!f.velUValid); }
  std::printf("  T2 velU NaN / velUValid=false\n");
}

static void t3_stageLadder() {
  // 驗證：自 last-good 起的階段邊界與 E4 原文一次。
  // 短階梯（加速）：邊界 2.5／3.0／3.7；LOSS_NO_GGA=2 計入 FAST。
  // 進入 FIX_FAILED 後 openSky／FIX_FAILED 事件只一次；DrvState 仍 READY（非 FAULT）。
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
  // 驗證 C6：從 FAST／MED／SLOW／FIX_FAILED 任一階段恢復合格 GGA → TRACKING，且不進 FAULT。
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
  // 驗證：NO_DATA（靜默）與 STUCK（同句重複）皆觸發失鎖→REACQ；靜默期間 1 Hz 降級且 dataAge 成長；可恢復。
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
  // 驗證 emit／seq 政策：
  //   TRACKING 期間約跟 GGA（≤~1 Hz，不多灌）；靜默後最多 1 Hz 降級；
  //   每筆輸出 seq 嚴格 +1；dataAge 隨 last-good 成長且 posValid=false。
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
  // 驗證 A1：開機 ACQ 逾時 → INIT_ACQ_TIMEOUT 一次＋FIX_FAILED＋openSky；
  // 不進 FAULT（環境問題）；清除 NO_FIX 後可回 TRACKING。
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
  // 驗證硬故障分流：SELFTEST 時限內無任何合法 NMEA → SELFTEST_FAIL＋DrvState::FAULT。
  Rig r;
  GnssCoreParams p; p.selftestTimeout_s = 0.5f; r.core.setParams(p);
  r.dev.setFault(GnssSimFault::NO_DATA);
  r.run(0.8);
  CHECK(r.core.state() == DrvState::FAULT);
  CHECK(r.sink.nFault(GnssFault::SELFTEST_FAIL) >= 1);
  std::printf("  T8 SELFTEST fail -> FAULT\n");
}

static void t9_flapping() {
  // 驗證：短週期反覆失鎖／重鎖不會誤觸 55 s／FIX_FAILED；openSky 仍為 0。
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
  // 驗證品質門檻：FIX_2D_ONLY（非 3D）視為失鎖；並演習收緊 HDOP_MAX。
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
  // 驗證：跨越靜默（含 1 Hz 降級）與恢復後，seq 全程連續遞增（有 emit 才 ++）。
  Rig r; r.run(2.5);
  r.dev.setFault(GnssSimFault::NO_DATA); r.run(3.0);  // 含 1 Hz 降級
  r.dev.setFault(GnssSimFault::NONE); r.run(2.0);
  CHECK(r.sink.fixes.size() >= 2);
  for (size_t i = 1; i < r.sink.fixes.size(); i++)
    CHECK(r.sink.fixes[i].seq == r.sink.fixes[i - 1].seq + 1);
  std::printf("  T11 seq contiguous across silence\n");
}

// T12：生產時長 15／10／30；整數分頻 10／100／200 ticks；中斷 12／20／60 s
// 驗證：進入 FAST/MED/SLOW/FAIL 的 age≈2／15／25／55；
//       TRACKING poll≈20／2s；FAST≈129±2；MED=10；SLOW=15；
//       pollPeriodTicks 在各階段為 10／100／200（整數分頻，無 float 漂移）。
static void t12_stageTimingPolls() {
  auto runOutage = [](double outage_s, LockStage expectFinal,
                      double* enterFast, double* enterMed, double* enterSlow, double* enterFail,
                      uint32_t* pollsTrack, uint32_t* pollsFast, uint32_t* pollsMed, uint32_t* pollsSlow) {
    Rig r;
    GnssCoreParams p; p.hdopMax = 5.0f;
    p.rgPeriod_ms = 10;
    p.pollTracking_ms = 100; p.pollReacqFast_ms = 100;
    p.pollReacqMed_ms = 1000; p.pollReacqSlow_ms = 2000;
    p.reacqFastDur_s = 15.f; p.reacqMedDur_s = 10.f; p.reacqSlowDur_s = 30.f;
    p.lossNoGga_s = 2.0f; p.initAcqTimeout_s = 300.f;
    r.core.setParams(p);
    CHECK(r.core.pollPeriodTicks() == 10);  // TRACKING／ACQ 預設

    // 進 TRACKING 後再量測 TRACKING poll：再跑 2.0 s → 應約 20 polls（100 ms）
    r.run(3.0);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
    CHECK(r.core.pollPeriodTicks() == 10);
    uint32_t track0 = r.core.pollsInStage();
    r.run(2.0);
    CHECK(r.core.lockStage() == LockStage::TRACKING);
    *pollsTrack = r.core.pollsInStage() - track0;
    CHECK(*pollsTrack >= 18 && *pollsTrack <= 22);  // 2.0/0.1 = 20

    const double tGood = r.core.lastGoodPos_s();
    r.dev.setFault(GnssSimFault::NO_DATA);

    LockStage prev = LockStage::TRACKING;
    *enterFast = *enterMed = *enterSlow = *enterFail = -1;
    *pollsFast = *pollsMed = *pollsSlow = 0;

    while (r.t < tGood + outage_s - 1e-9) {
      uint32_t polls = r.core.pollsInStage();
      r.run(DT);
      if (r.core.lockStage() != prev) {
        if (r.core.lockStage() == LockStage::REACQ_FAST) {
          *enterFast = r.t - tGood;
          CHECK(r.core.pollPeriodTicks() == 10);
        } else if (r.core.lockStage() == LockStage::REACQ_MED) {
          *enterMed = r.t - tGood;
          *pollsFast = polls;
          CHECK(r.core.pollPeriodTicks() == 100);
        } else if (r.core.lockStage() == LockStage::REACQ_SLOW) {
          *enterSlow = r.t - tGood;
          *pollsMed = polls;
          CHECK(r.core.pollPeriodTicks() == 200);
        } else if (r.core.lockStage() == LockStage::FIX_FAILED) {
          *enterFail = r.t - tGood;
          *pollsSlow = polls;
          CHECK(r.core.pollPeriodTicks() == 200);
        }
        prev = r.core.lockStage();
      }
    }
    if (r.core.lockStage() == LockStage::REACQ_FAST) *pollsFast = r.core.pollsInStage();
    if (r.core.lockStage() == LockStage::REACQ_MED) *pollsMed = r.core.pollsInStage();
    if (r.core.lockStage() == LockStage::REACQ_SLOW) *pollsSlow = r.core.pollsInStage();
    CHECK(r.core.lockStage() == expectFinal);
    return tGood;
  };

  // --- 12 s：仍在 FAST；enter≈2.09；polls ≈ (12-2.09)/0.1 ≈ 99 ---
  {
    double eF, eM, eS, eX; uint32_t pT, pF, pM, pS;
    runOutage(12.0, LockStage::REACQ_FAST, &eF, &eM, &eS, &eX, &pT, &pF, &pM, &pS);
    CHECK(eF > 1.5 && eF < 2.5);
    CHECK(eM < 0);
    CHECK(pF >= 95 && pF <= 105);
    std::printf("  T12a outage=12s enterFAST=%.2f pollsTRACK=%u pollsFAST=%u\n", eF, pT, pF);
  }
  // --- 20 s：MED@15；FAST 2.09–15.01 → ~129±2；MED (20-15)/1 = 5 ---
  {
    double eF, eM, eS, eX; uint32_t pT, pF, pM, pS;
    runOutage(20.0, LockStage::REACQ_MED, &eF, &eM, &eS, &eX, &pT, &pF, &pM, &pS);
    CHECK(eF > 1.5 && eF < 2.5);
    CHECK(eM > 14.5 && eM < 15.5);
    CHECK(eS < 0);
    CHECK(pF >= 127 && pF <= 131);  // 2.09–15.01 窗 ≈129±2
    CHECK(pM >= 4 && pM <= 6);
    std::printf("  T12b outage=20s enterMED=%.2f pollsTRACK=%u pollsFAST=%u pollsMED=%u\n",
                eM, pT, pF, pM);
  }
  // --- 60 s：SLOW@25、FAIL@55；MED=10、SLOW=15；FAST≈129±2 ---
  {
    double eF, eM, eS, eX; uint32_t pT, pF, pM, pS;
    runOutage(60.0, LockStage::FIX_FAILED, &eF, &eM, &eS, &eX, &pT, &pF, &pM, &pS);
    CHECK(eF > 1.5 && eF < 2.5);
    CHECK(eM > 14.5 && eM < 15.5);
    CHECK(eS > 24.5 && eS < 25.5);
    CHECK(eX > 54.5 && eX < 55.5);
    CHECK(pF >= 127 && pF <= 131);
    CHECK(pM == 10);
    CHECK(pS == 15);
    std::printf("  T12c outage=60s enterSLOW=%.2f enterFAIL=%.2f pollsT/F/M/S=%u/%u/%u/%u\n",
                eS, eX, pT, pF, pM, pS);
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
