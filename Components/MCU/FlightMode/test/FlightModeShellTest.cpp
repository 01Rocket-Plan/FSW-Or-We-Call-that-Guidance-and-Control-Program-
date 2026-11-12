// FlightMode 殼測試：只驗「殼是安全的」——不論輸入多誇張，都不離開初始模式、不分離、不開傘。
// 填完 FM-xx 之後，這支要換成 SSS test_flight_mode.py 的 COVER4 站。
#include "Components/MCU/FlightMode/FlightModeCore.hpp"

#include <cstdio>

using namespace Mcu;

static int g_fail = 0;
#define CHECK(c, msg)                                      \
  do {                                                     \
    if (!(c)) {                                            \
      std::printf("  FAIL %s (line %d)\n", msg, __LINE__); \
      ++g_fail;                                            \
    }                                                      \
  } while (0)

static void run(FmRole role, FmMode expect) {
  FlightModeCore c;
  c.setRole(role);
  FmInputs in;
  FmGroundCmd cmd;
  FmOutputs o;
  // 最壞輸入：全部「放行」、大加速度、大傾角、地面 ARM＋FIRE＋中止
  in.haveState = in.imu_fresh = in.velValid = true;
  in.seated = in.loop_ok = in.ready = in.ground_link = true;
  in.sep_switch_open = in.sep_ok = in.contact_ds = in.contact_us = true;
  in.link_alive = in.link_sep_cmd = true;
  in.accel_u_mps2 = 100.0f;
  in.tilt_rad = 3.0f;
  in.rate_rads = 50.0f;
  cmd.arm = cmd.fire = cmd.abortValid = true;
  for (int k = 0; k < 60000; ++k) {  // 60 s
    in.t_s = k * 1e-3;
    c.tick(in, cmd, o);
    if (o.mode != expect || o.sepFire || o.chuteDs || o.chuteUs || o.nEvents != 0) {
      CHECK(false, "殼不得轉移、分離、開傘或發事件");
      return;
    }
  }
}

int main() {
  std::printf("ST-FM-SHELL-SAFE-DS\n");
  run(FmRole::DS, FmMode::PAD_IDLE);
  std::printf("ST-FM-SHELL-SAFE-US\n");
  run(FmRole::US, FmMode::US_WAIT);
  if (g_fail) {
    std::printf("FlightMode shell: FAIL (%d)\n", g_fail);
    return 1;
  }
  std::printf("FlightMode shell: PASS\n");
  return 0;
}
