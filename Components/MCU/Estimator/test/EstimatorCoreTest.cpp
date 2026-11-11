// EstimatorCore 主機測試（輕量；CONTRIBUTING 第 10 條）
// 六站：墊上對準、傾斜對準、陀螺積分、垂直 KF、IMU 過期、氣壓離群
#include "Components/MCU/Estimator/EstimatorCore.hpp"

#include <cmath>
#include <cstdio>

using namespace Mcu;

static int g_fail = 0;
#define CHECK(c, msg)                                   \
  do {                                                  \
    if (!(c)) {                                         \
      std::printf("  FAIL %s (line %d)\n", msg, __LINE__); \
      ++g_fail;                                         \
    }                                                   \
  } while (0)

static const float G = 9.80665f;
static const double DT = 1e-3;

struct Sim {
  EstimatorCore est;
  StateOutC out;
  double t = 0;
  uint32_t imuSeq = 0, baroSeq = 0;
  void tick(const float f[3], const float w[3], bool pushBaro = false, float baroAlt = 0, bool imu = true) {
    t += DT;
    if (imu) {
      ImuIn s;
      s.t_s = t;
      s.seq = ++imuSeq;
      s.valid = true;
      for (int i = 0; i < 3; ++i) {
        s.acc_mps2[i] = f[i];
        s.gyro_rads[i] = w[i];
      }
      est.pushImu(s);
    }
    if (pushBaro) {
      BaroIn b;
      b.t_s = t;
      b.seq = ++baroSeq;
      b.valid = true;
      b.alt_m = baroAlt;
      est.pushBaro(b);
    }
    est.step(t, out);
  }
  void pad(double seconds, const float f[3], const float w[3], float baroAlt = 50.0f) {
    const int n = static_cast<int>(seconds / DT);
    for (int k = 0; k < n; ++k) tick(f, w, (k % 10) == 0, baroAlt);
  }
};

static void testPadAlign() {
  std::printf("ST-EST-PAD-ALIGN\n");
  Sim s;
  s.est.setOnPad(true);
  const float f[3] = {G, 0, 0};
  const float w[3] = {0.002f, -0.001f, 0.0005f};  // 小零偏
  s.pad(0.3, f, w);
  CHECK(!s.out.imuValid, "未滿對準時間不得 imuValid");
  s.pad(1.0, f, w);
  CHECK(s.est.diag().aligned, "1.3 s 後應已對準");
  CHECK(s.out.imuValid, "對準後 imuValid");
  CHECK(s.out.tilt_rad < 1e-5f, "直立 tilt≈0（atan2，數值穩定）");
  CHECK(std::fabs(s.out.alt_m) < 0.05f, "墊上 alt≈0");
  CHECK(std::fabs(s.out.vel_enu_mps[2]) < 0.01f, "墊上 vU≈0");
  CHECK(std::fabs(s.out.accAxial_mps2) < 0.05f, "墊上 a_u≈0");
  CHECK(std::fabs(s.est.diag().gyroBias_rads[0] - 0.002f) < 2e-4f, "零偏 x 估到");
  CHECK(std::fabs(s.out.rate_body_rads[0]) < 2e-4f, "扣零偏後角速度≈0");
}

static void testTiltAlign() {
  std::printf("ST-EST-TILT-ALIGN\n");
  Sim s;
  s.est.setOnPad(true);
  const float a = 5.0f * 3.14159265f / 180.0f;  // 導軌傾 5°（仰角 85°，標稱發射角）
  const float f[3] = {G * std::cos(a), 0, G * std::sin(a)};
  const float w[3] = {0, 0, 0};
  s.pad(1.0, f, w);
  CHECK(std::fabs(s.out.tilt_rad - a) < 1e-3f, "傾 5°（仰角 85°）對準");
  CHECK(std::fabs(s.out.accAxial_mps2) < 0.05f, "傾斜靜置 a_u≈0");
}

static void testPadLeveling() {
  std::printf("ST-EST-PAD-LEVEL\n");
  Sim s;
  s.est.setOnPad(true);
  const float f0[3] = {G, 0, 0};
  const float w0[3] = {0, 0, 0};
  s.pad(1.0, f0, w0);
  // 對準後導軌被扳到 5°（陀螺沒看到轉動）：只能靠加速度修正收斂；符號反了會發散
  const float a = 5.0f * 3.14159265f / 180.0f;
  const float f[3] = {G * std::cos(a), G * std::sin(a), 0};
  s.pad(10.0, f, w0);
  CHECK(std::fabs(s.out.tilt_rad - a) < 2e-3f, "墊上水平修正收斂到 5°");
}

static void testGyroIntegration() {
  std::printf("ST-EST-GYRO\n");
  Sim s;
  s.est.setOnPad(true);
  const float f0[3] = {G, 0, 0};
  const float w0[3] = {0, 0, 0};
  s.pad(1.0, f0, w0);
  s.est.setOnPad(false);  // 離台：純陀螺
  const float w[3] = {0, 0.1f, 0};  // 繞機體 Y（俯仰）0.1 rad/s
  const float f[3] = {0, 0, 0};     // 自由飛（不影響姿態）
  for (int k = 0; k < 1000; ++k) s.tick(f, w);
  CHECK(std::fabs(s.out.tilt_rad - 0.1f) < 2e-3f, "1 s × 0.1 rad/s 俯仰 → tilt 0.1 rad");
  // 滾轉（繞機體 X＝鼻）不改傾角
  const float wr[3] = {0.5f, 0, 0};
  const float before = s.out.tilt_rad;
  for (int k = 0; k < 1000; ++k) s.tick(f, wr);
  CHECK(std::fabs(s.out.tilt_rad - before) < 1e-3f, "滾轉 0.5 rad/s × 1 s 不改傾角");
}

static void testVerticalKf() {
  std::printf("ST-EST-VERT-KF\n");
  Sim s;
  s.est.setOnPad(true);
  const float f0[3] = {G, 0, 0};
  const float w0[3] = {0, 0, 0};
  s.pad(1.0, f0, w0, 50.0f);
  s.est.setOnPad(false);
  // 1 s 等加速 a_u = 20 m/s²；氣壓照真值給
  const float aU = 20.0f;
  const float f[3] = {G + aU, 0, 0};
  for (int k = 1; k <= 1000; ++k) {
    const double tt = k * DT;
    const float h = static_cast<float>(0.5 * aU * tt * tt);
    s.tick(f, w0, (k % 10) == 0, 50.0f + h);
  }
  CHECK(std::fabs(s.out.vel_enu_mps[2] - 20.0f) < 0.3f, "v_u≈20 m/s");
  CHECK(std::fabs(s.out.alt_m - 10.0f) < 0.3f, "h≈10 m");
  CHECK(std::fabs(s.out.accAxial_mps2 - aU) < 1e-3f, "a_u = 20");
  CHECK(s.out.velValid, "velValid");
}

static void testImuStale() {
  std::printf("ST-EST-IMU-STALE\n");
  Sim s;
  s.est.setOnPad(true);
  const float f0[3] = {G, 0, 0};
  const float w0[3] = {0, 0, 0};
  s.pad(1.0, f0, w0);
  CHECK(s.out.imuValid, "前提：新鮮");
  for (int k = 0; k < 19; ++k) s.tick(f0, w0, false, 0, false);
  CHECK(s.out.imuValid, "19 ms 仍新鮮");
  for (int k = 0; k < 3; ++k) s.tick(f0, w0, false, 0, false);
  CHECK(!s.out.imuValid, ">20 ms 不新鮮");
  CHECK(s.est.diag().imuStaleCount == 1, "過期計一次");
  s.tick(f0, w0);
  CHECK(s.out.imuValid, "恢復");
}

static void testBaroOutlier() {
  std::printf("ST-EST-BARO-GATE\n");
  Sim s;
  s.est.setOnPad(true);
  const float f0[3] = {G, 0, 0};
  const float w0[3] = {0, 0, 0};
  s.pad(1.0, f0, w0, 50.0f);
  s.est.setOnPad(false);
  const float fFree[3] = {G, 0, 0};  // 懸停：a_u = 0
  s.tick(fFree, w0, true, 50.0f + 100.0f);  // 跳 100 m
  CHECK(s.est.diag().baroRejectCount == 1, "離群被丟");
  CHECK(std::fabs(s.out.alt_m) < 0.05f, "高度沒被拉走");
}

int main() {
  testPadAlign();
  testTiltAlign();
  testPadLeveling();
  testGyroIntegration();
  testVerticalKf();
  testImuStale();
  testBaroOutlier();
  if (g_fail) {
    std::printf("EstimatorCore: FAIL (%d)\n", g_fail);
    return 1;
  }
  std::printf("EstimatorCore: PASS\n");
  return 0;
}
