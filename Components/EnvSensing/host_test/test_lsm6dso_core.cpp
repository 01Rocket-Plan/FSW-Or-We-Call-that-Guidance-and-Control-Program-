// LSM6DSO 備援核心 host 單測：lsm6dso_core.c ＋ 暫存器級 Lsm6dsoSimDevice（同一份給 F´ Sim 用）
#include "../Lsm6dsoDriver/lsm6dso_core.h"
#include "../Lsm6dsoDriver/Lsm6dsoSimDevice.hpp"
#include <cmath>
#include <cstdio>

using Components::Lsm6dsoSimDevice;
static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); g_fail++; } \
  else { std::printf("ok   %s\n", msg); } } while (0)
static const float G0 = 9.80665f;

// 跑到 READY（或上限拍數）；回傳拍數
static int run_until(LsmCore* c, double* t, LsmSampleOut* out, LsmDrvState want, int maxTicks) {
  for (int i = 0; i < maxTicks; ++i) {
    *t += 0.001; lsm_core_tick(c, *t, out);
    if (lsm_core_state(c) == want) return i + 1;
    if (lsm_core_state(c) == LSM_DRV_FAULT && want != LSM_DRV_FAULT) return -1;
  }
  return -1;
}

static void test_happy() {
  Lsm6dsoSimDevice dev; LsmCore* c = lsm_core_create(dev.busOps());
  LsmSampleOut out; double t = 0;
  int n = run_until(c, &t, &out, LSM_DRV_READY, 5000);
  std::printf("     READY after %d ticks (~%.2f s)\n", n, n / 1000.0);
  CHECK(n > 0, "reach READY（SW_RESET → WHO_AM_I 0x6C → ST_XL/ST_G 自測 → 靜置）");
  uint32_t prev = lsm_core_seq(c);
  for (int i = 0; i < 100; ++i) {
    t += 0.001; lsm_core_tick(c, t, &out);
    if (!out.valid || out.seq != prev + 1) break;
    prev = out.seq;
  }
  CHECK(out.valid && lsm_core_state(c) == LSM_DRV_READY, "READY 持續出有效樣本（抖動不觸發卡值）");
  CHECK(std::fabs(out.acc_mps2[0] - G0) < 0.3f, "機體 X ≈ +1 g（墊上，晶片 Z 沿鼻）");
  CHECK(std::fabs(out.acc_mps2[1]) < 0.3f && std::fabs(out.acc_mps2[2]) < 0.3f, "機體 Y/Z ≈ 0");
  CHECK(std::fabs(out.gyro_rads[0]) < 0.01f && std::fabs(out.gyro_rads[2]) < 0.01f, "靜置角速度 ≈ 0");
  CHECK(!out.gyroSat && !out.accelSat, "靜置不飽和");
  lsm_core_destroy(c);
}

static void test_no_valid_early() {
  Lsm6dsoSimDevice dev; LsmCore* c = lsm_core_create(dev.busOps());
  LsmSampleOut out; double t = 0; bool anyValid = false;
  for (int i = 0; i < 5000 && lsm_core_state(c) != LSM_DRV_READY; ++i) {
    t += 0.001; lsm_core_tick(c, t, &out); if (out.valid) anyValid = true;
  }
  CHECK(!anyValid, "READY 之前一筆有效樣本都沒有");
  lsm_core_destroy(c);
}

static void test_whoami() {
  Lsm6dsoSimDevice dev; dev.setInject(4); LsmCore* c = lsm_core_create(dev.busOps());
  LsmSampleOut out; double t = 0;
  CHECK(run_until(c, &t, &out, LSM_DRV_FAULT, 200) > 0, "WHO_AM_I 錯 → FAULT");
  CHECK(lsm_core_fault(c) == LSM_FAULT_WHOAMI, "fault reason = WHOAMI(1)");
  CHECK(!out.valid, "FAULT 不出有效樣本");
  lsm_core_destroy(c);
}

static void test_wrong_addr() {
  Lsm6dsoSimDevice dev; LsmCore* c = lsm_core_create(dev.busOps());
  LsmParams p = lsm_params_default(); p.i2cAddr = LSM6DSO_I2C_ADDR_SA0_HIGH; lsm_core_set_params(c, &p);
  LsmSampleOut out; double t = 0;
  CHECK(run_until(c, &t, &out, LSM_DRV_FAULT, 200) > 0 && lsm_core_fault(c) == LSM_FAULT_WHOAMI,
        "I2C 位址設錯（0x6B，晶片在 0x6A）→ WHOAMI FAULT");
  lsm_core_destroy(c);
}

static void test_selftest_fail() {
  Lsm6dsoSimDevice dev; dev.setInject(6); LsmCore* c = lsm_core_create(dev.busOps());
  LsmSampleOut out; double t = 0;
  CHECK(run_until(c, &t, &out, LSM_DRV_FAULT, 5000) > 0, "自測響應太弱（10 mg < 50 mg）→ FAULT");
  CHECK(lsm_core_fault(c) == LSM_FAULT_SELFTEST, "fault reason = SELFTEST(4)");
  lsm_core_destroy(c);
}

static void test_bus_err() {
  Lsm6dsoSimDevice dev; dev.setInject(3); LsmCore* c = lsm_core_create(dev.busOps());
  LsmSampleOut out; double t = 0;
  CHECK(run_until(c, &t, &out, LSM_DRV_FAULT, 100) > 0, "匯流排一直錯 → FAULT");
  CHECK(lsm_core_fault(c) == LSM_FAULT_BUS && lsm_core_bus_err(c) >= 5, "fault reason = BUS(2)，busErrCount ≥ 門檻");
  lsm_core_destroy(c);
}

static void test_bus_err_in_ready() {
  Lsm6dsoSimDevice dev; LsmCore* c = lsm_core_create(dev.busOps());
  LsmSampleOut out; double t = 0;
  run_until(c, &t, &out, LSM_DRV_READY, 5000);
  dev.setInject(3);
  CHECK(run_until(c, &t, &out, LSM_DRV_FAULT, 20) > 0 && lsm_core_fault(c) == LSM_FAULT_BUS, "READY 中匯流排斷 → BUS FAULT");
  lsm_core_destroy(c);
}

static void test_stuck() {
  Lsm6dsoSimDevice dev; LsmCore* c = lsm_core_create(dev.busOps());
  LsmSampleOut out; double t = 0;
  run_until(c, &t, &out, LSM_DRV_READY, 5000);
  dev.setInject(1);
  CHECK(run_until(c, &t, &out, LSM_DRV_FAULT, 200) > 0, "數值凍結 → FAULT");
  CHECK(lsm_core_fault(c) == LSM_FAULT_STUCK && lsm_core_stuck(c) >= 1, "fault reason = STUCK(3)");
  lsm_core_destroy(c);
}

// 飽和：核心每拍都標 gyroSat／accelSat；F´ 外殼只在「剛開始飽和」那拍記事件——這裡用同一條閂鎖邏輯驗
static void test_saturation_onset() {
  Lsm6dsoSimDevice dev; LsmCore* c = lsm_core_create(dev.busOps());
  LsmParams p = lsm_params_default(); p.stuckN = 100000; lsm_core_set_params(c, &p);
  LsmSampleOut out; double t = 0;
  run_until(c, &t, &out, LSM_DRV_READY, 5000);
  bool gLatch = false, aLatch = false; int gEvents = 0, aEvents = 0;
  auto step = [&](uint8_t inj, int n) {
    dev.setInject(inj);
    for (int i = 0; i < n; ++i) {
      t += 0.001; lsm_core_tick(c, t, &out);
      if (out.gyroSat && !gLatch) gEvents++;
      if (out.accelSat && !aLatch) aEvents++;
      if (out.valid) { gLatch = out.gyroSat; aLatch = out.accelSat; }
    }
  };
  step(2, 50);   // 陀螺削頂 32767 LSB × 70 mdps ≈ 2294 dps ≈ 40 rad/s ≥ 34
  CHECK(out.valid && out.gyroSat, "陀螺飽和旗標");
  CHECK(gEvents == 1, "連續 50 拍飽和只記 1 次 GyroSaturated");
  step(0, 10); step(2, 10);
  CHECK(gEvents == 2, "解除後再飽和 → 再記 1 次");
  step(0, 10);
  step(5, 50);   // 加速度削頂 32767 × 0.488 mg ≈ 15.99 g ≈ 156.8 m/s² ≥ 156
  CHECK(out.valid && out.accelSat, "加速度飽和旗標");
  CHECK(aEvents == 1, "連續 50 拍飽和只記 1 次 AccelSaturated");
  lsm_core_destroy(c);
}

int main() {
  test_happy();
  test_no_valid_early();
  test_whoami();
  test_wrong_addr();
  test_selftest_fail();
  test_bus_err();
  test_bus_err_in_ready();
  test_stuck();
  test_saturation_onset();
  std::printf("%s (%d fail)\n", g_fail ? "LSM6DSO TESTS FAILED" : "ALL LSM6DSO TESTS PASSED", g_fail);
  return g_fail ? 1 : 0;
}
