// =============================================================================
// GnssHostTest — GnssSimDevice 本身的主機測試（不需 F´；FakeGnssCore 只收句）
//
// 與 GnssCoreTest 分工：本檔驗證「模擬器位元組行為」；Core 狀態機在 GnssCoreTest。
//   T9a 名義：校驗和正確、四句型齊全、~1 Hz
//   T9b 100 Hz tick 仍 1 Hz 輸出；latency 內不出位元組
//   T9c baud 限速：每 tick 位元組 ≤ baud/10*dt
//   T9d 雜訊：位置誤差在 σ 範圍且非精確真值
//   T9e 故障注入：NO_FIX／2D／BAD_CHECKSUM／TRUNCATED／NO_DATA／STUCK
// =============================================================================
#include "Components/Gnss/GnssDriverSim/GnssSimDevice.hpp"
#include "FakeGnssCore.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace Gnss;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)
static const double kDeg = 3.14159265358979323846 / 180.0;

struct Rig {
  GnssSimDevice dev; FakeGnssCore core{dev}; double dt;
  explicit Rig(double dt_s = 1.0) : dt(dt_s) {
    const float v[3] = {10.f, 5.f, 2.f};
    dev.setTrajectory(24.0 * kDeg, 121.0 * kDeg, 50.0, v);
  }
  void run(int n) { for (int i = 0; i < n; i++) { dev.advance(dt); core.tick(); } }
};

static std::vector<std::string> fields(const std::string& s) {
  std::vector<std::string> f; std::string cur;
  for (size_t i = 1; i < s.size() && s[i] != '*'; i++) { if (s[i] == ',') { f.push_back(cur); cur.clear(); } else cur += s[i]; }
  f.push_back(cur); return f;
}
static double parseLat(const std::string& v) { double x = std::atof(v.c_str()); int d = static_cast<int>(x / 100); return (d + (x - d * 100) / 60.0) * kDeg; }

// T9a：所有句子校驗和正確，四種句型齊全，1 Hz 節奏
static void t9_nominal() {
  Rig r; r.run(10);
  CHECK(r.core.badChecksum == 0 && r.core.truncated == 0);
  // 延遲 0.15 s、tick 1 s：第 1 s 的 epoch 在第 2 個 tick 才送出 → 10 個 tick 收到 9 組
  CHECK(r.core.count("GPGGA") == 9 && r.core.count("GPRMC") == 9 && r.core.count("GPVTG") == 9 && r.core.count("GPGSA") == 9);
  CHECK(r.dev.epochsGenerated() == 10);
  for (auto& s : r.core.good) CHECK(s.size() <= 82);
}
// T9b：tick 改 100 Hz 也維持 1 Hz 輸出；延遲生效（epoch 後 0.15 s 內不出位元組）
static void t9_cadenceLatency() {
  Rig r(0.01); r.dev.setLatency(0.15);
  r.run(114);  // 1.14 s：第一個 epoch(1.0 s) 尚未釋出
  CHECK(r.core.bytes == 0);
  r.run(100);  // 2.14 s
  CHECK(r.core.count("GPGGA") == 1);
  r.run(800);  // 10.14 s
  CHECK(r.core.count("GPGGA") == 9);
}
// T9c：baud 限速，每個 tick 不超過 baud/10*dt 位元組
static void t9_baud() {
  Rig r(0.01); uint32_t prev = 0, maxPerTick = 0;
  for (int i = 0; i < 500; i++) { r.run(1); uint32_t d = r.core.bytes - prev; prev = r.core.bytes; if (d > maxPerTick) maxPerTick = d; }
  CHECK(maxPerTick <= 10);  // 9600 baud × 0.01 s / 10 bit
  CHECK(r.core.count("GPGGA") >= 3);
}
// T9d：量測有雜訊但在範圍內（位置誤差 < 5σ，且不等於真值）
static void t9_noise() {
  Rig r; r.run(30); int exact = 0; double maxErr = 0; double t = 1.0;
  for (auto& s : r.core.good) {
    if (s.compare(1, 5, "GPGGA") != 0) continue;
    auto f = fields(s); double lat = parseLat(f[2]), tl, tn, ta; r.dev.truthAt(t, tl, tn, ta);
    double err = std::fabs(lat - tl) * 6378137.0; if (err > maxErr) maxErr = err; if (err < 1e-3) exact++;
    double alt = std::atof(f[9].c_str()); CHECK(std::fabs(alt - ta) < 20.0);
    t += 1.0;
  }
  std::printf("  max north err %.2f m\n", maxErr);
  CHECK(maxErr < 12.5 && maxErr > 0.1 && exact == 0);
}
// T9e：各種故障注入
static void t9_faults() {
  { Rig r; r.dev.setFault(GnssSimFault::NO_FIX); r.run(5);
    for (auto& s : r.core.good) { auto f = fields(s);
      if (f[0] == "GPGGA") CHECK(f[6] == "0"); if (f[0] == "GPGSA") CHECK(f[2] == "1"); if (f[0] == "GPRMC") CHECK(f[2] == "V"); } }
  { Rig r; r.dev.setFault(GnssSimFault::FIX_2D_ONLY); r.run(5); size_t k = 0;
    for (auto& s : r.core.good) { auto f = fields(s); if (f[0] == "GPGSA") { CHECK(f[2] == "2"); k++; } } CHECK(k == 4); }
  { Rig r; r.dev.setFault(GnssSimFault::BAD_CHECKSUM); r.run(5);
    CHECK(r.core.badChecksum == 4 && r.core.count("GPGGA") == 0 && r.core.count("GPRMC") == 4); }
  { Rig r; r.dev.setFault(GnssSimFault::TRUNCATED); r.run(5);
    CHECK(r.core.truncated == 4 && r.core.count("GPGGA") == 0 && r.core.count("GPRMC") == 4); }
  { Rig r; r.run(3); uint32_t b = r.core.bytes; r.dev.setFault(GnssSimFault::NO_DATA); r.run(5);
    CHECK(r.core.bytes == b);
    r.dev.setFault(GnssSimFault::NONE); r.run(3); CHECK(r.core.bytes > b); }  // 恢復後會再送
  { Rig r; r.run(2); r.dev.setFault(GnssSimFault::STUCK); r.run(5); std::vector<std::string> g;
    for (auto& s : r.core.good) if (s.compare(1, 5, "GPGGA") == 0) g.push_back(s);
    CHECK(g.size() >= 5); for (size_t i = 2; i < g.size(); i++) CHECK(g[i] == g[1]); }
}
// T9f：偶發掉 fix、HDOP／衛星數會變動
static void t9_variation() {
  Rig r; r.dev.setDropoutProb(0.2f); r.run(200); int q0 = 0; std::vector<std::string> hd, ns;
  for (auto& s : r.core.good) { auto f = fields(s); if (f[0] != "GPGGA") continue; if (f[6] == "0") q0++; else { hd.push_back(f[8]); ns.push_back(f[7]); } }
  std::printf("  dropouts %d / 199\n", q0);
  CHECK(q0 > 10 && q0 < 80);
  bool hdVar = false, nsVar = false; for (size_t i = 1; i < hd.size(); i++) { if (hd[i] != hd[0]) hdVar = true; if (ns[i] != ns[0]) nsVar = true; }
  CHECK(hdVar && nsVar);
}

int main() {
  t9_nominal(); t9_cadenceLatency(); t9_baud(); t9_noise(); t9_faults(); t9_variation();
  std::printf(g_fail ? "GnssHostTest: %d FAIL\n" : "GnssHostTest: ALL PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
