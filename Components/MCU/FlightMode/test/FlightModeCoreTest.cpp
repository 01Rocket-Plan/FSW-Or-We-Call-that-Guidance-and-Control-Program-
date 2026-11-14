// FlightModeCore 主機測試：SSS sss/selftest/test_flight_mode.py 的檢查站（CP-*）移植，
// 加上 repo 差異（發射架安檢 FM-03、架點火證據 FM-08、臍帶拔脫）的案例。
// 分離開關、著地接觸用模擬值餵（SIL 不需要硬體）。
#include "Components/MCU/FlightMode/FlightModeCore.hpp"

#include <cmath>
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

namespace {

struct Sim {
  FlightModeCore c;
  FmInputs in;
  FmGroundCmd cmd;
  FmOutputs o;
  double t = 0.0;
  int nSep = 0, nChuteDs = 0, nChuteUs = 0, nSide = 0;
  int ev[16] = {};
  uint8_t lastMask = 0;
  double tSep = -1, tChuteDs = -1;

  explicit Sim(FmRole r = FmRole::DS) {
    c.setRole(r);
    in.imu_fresh = in.haveState = true;
  }
  void step(int n = 1) {
    for (int k = 0; k < n; ++k) {
      in.t_s = t;
      c.tick(in, cmd, o);
      cmd = FmGroundCmd();  // 地面指令單拍
      for (int i = 0; i < o.nEvents; ++i) ++ev[static_cast<int>(o.events[i])];
      if (o.sepFire) { ++nSep; lastMask = o.sepMask; if (tSep < 0) tSep = t; }
      if (o.chuteDs) { ++nChuteDs; if (tChuteDs < 0) tChuteDs = t; }
      if (o.chuteDsSide) ++nSide;
      if (o.chuteUs) ++nChuteUs;
      t = std::round((t + 1e-3) * 1e6) / 1e6;  // 不累積浮點誤差
    }
  }
  // 臍帶插著、架有收到箭、架在某狀態
  void pad(uint8_t st, uint8_t extraFlags = 0, float cd = 0.0f) {
    in.padLinkOk = true;
    in.padState = st;
    in.padFlags = static_cast<uint8_t>(FmPad::FLAG_ROCKET_LINK_OK | extraFlags);
    in.padCountdown_s = cd;
  }
  void umbilicalGone() { in.padLinkOk = false; }  // 臍帶拔脫 → UmbilicalLink 標 linkOk=false
  FmMode ds() const { return c.state().ds; }
  int n(FmEvent e) const { return ev[static_cast<int>(e)]; }
};

const float kDeg = 3.14159265f / 180.0f;

// 推到 ARMED
void toArmed(Sim& s) {
  s.pad(FmPad::ARMED);
  s.step();
}

}  // namespace

// ---------------------------------------------------------------- FM-03／07
static void cpInterlock() {
  std::printf("CP-INTERLOCK（架安檢）\n");
  {
    Sim s;
    s.pad(FmPad::SAFE);
    s.step(20);
    CHECK(s.ds() == FmMode::PAD_IDLE, "架 SAFE 不得 ARMED");
  }
  {
    Sim s;
    s.pad(FmPad::ARMED);
    s.in.padLinkOk = false;
    s.step(20);
    CHECK(s.ds() == FmMode::PAD_IDLE, "架狀態過期不得 ARMED");
  }
  {
    Sim s;
    s.pad(FmPad::ARMED);
    s.in.padFlags = 0;  // 架沒收到箭
    s.step(20);
    CHECK(s.ds() == FmMode::PAD_IDLE, "架沒收到箭（ROCKET_LINK_OK=0）不得 ARMED");
  }
  {
    Sim s;
    s.pad(FmPad::IGNITION, FmPad::FLAG_IGNITER_ON);
    s.in.padFlags = FmPad::FLAG_IGNITER_ON;  // 架點火但箭不在流程（不該發生）
    s.in.accel_u_mps2 = 30.0f;
    s.step(100);
    CHECK(s.ds() == FmMode::PAD_IDLE, "PAD_IDLE 看到點火不得 LIFTOFF");
    CHECK(s.n(FmEvent::DROP_FIRE) > 0, "PAD_IDLE 看到點火要記 DROP_FIRE");
  }
  {
    Sim s;
    toArmed(s);
    CHECK(s.ds() == FmMode::ARMED, "架 ARMED＋鏈路新 → ARMED");
    s.pad(FmPad::COUNTDOWN, 0, 20.0f);
    s.step(100);
    CHECK(s.ds() == FmMode::ARMED, "倒數中維持 ARMED");
    s.pad(FmPad::SAFE);  // 架任一紅燈／ABORT／DISARM
    s.step();
    CHECK(s.ds() == FmMode::PAD_IDLE, "架退 SAFE → PAD_IDLE");
  }
  {
    Sim s;
    toArmed(s);
    s.pad(FmPad::COUNTDOWN, 0, 0.5f);  // 還沒到最後一幀
    s.step(10);
    s.umbilicalGone();
    s.step();
    CHECK(s.ds() == FmMode::PAD_IDLE, "點火前臍帶斷 → PAD_IDLE");
  }
}

// ---------------------------------------------------------------- FM-08／08b
static void cpHangfire() {
  std::printf("CP-HANGFIRE\n");
  {
    Sim s;
    toArmed(s);
    s.pad(FmPad::IGNITION, FmPad::FLAG_IGNITER_ON);
    s.in.accel_u_mps2 = 0.0f;
    s.step(1100);
    CHECK(s.ds() == FmMode::HANGFIRE, "點火 1 s 沒離台 → HANGFIRE");
    CHECK(!s.c.state().motor_on, "HANGFIRE motor_on=false");
    s.step(10);
    CHECK(s.ds() == FmMode::HANGFIRE && s.n(FmEvent::DROP_FIRE) > 0, "HANGFIRE 不重試、記 DROP_FIRE");
    CHECK(s.nSep == 0 && s.nChuteDs == 0, "HANGFIRE 不分離不開傘");
  }
  {
    Sim s;
    toArmed(s);
    s.pad(FmPad::IGNITION, FmPad::FLAG_IGNITER_ON);
    s.step(30);
    s.pad(FmPad::HANG_FIRE);  // 架判定沒推力
    s.step();
    CHECK(s.ds() == FmMode::HANGFIRE, "架回報 HANG_FIRE → 立刻 HANGFIRE");
  }
  {
    Sim s;
    toArmed(s);
    s.pad(FmPad::IGNITION, FmPad::FLAG_IGNITER_ON);
    s.step(30);
    s.pad(FmPad::HOLD_ABORT);  // 有推力但沒鬆（LATE_RELEASE）
    s.step();
    CHECK(s.ds() == FmMode::HANGFIRE, "架回報 HOLD_ABORT → HANGFIRE");
  }
}

// ---------------------------------------------------------------- FM-08：倒數最後一幀＋臍帶拔脫
static void cpFinalCountdown() {
  std::printf("CP-FINAL-COUNT（臍帶 10 Hz，點火幀可能收不到）\n");
  {
    Sim s;
    toArmed(s);
    s.pad(FmPad::COUNTDOWN, 0, 0.1f);  // 最後一幀
    s.step();
    CHECK(s.c.state().fire_latched, "倒數剩 0.1 s → 先閂 FIRE");
    CHECK(std::fabs(s.c.state().t_fire_s - (s.t - 1e-3 + 0.1)) < 1e-6, "t_fire＝推回的 T=0");
    s.step(99);    // T=0 前沒有新幀
    s.umbilicalGone();  // 點火幀沒到就被拔掉
    s.in.accel_u_mps2 = 30.0f;
    s.step(40);
    CHECK(s.ds() == FmMode::BOOST_PASSIVE || s.ds() == FmMode::LIFTOFF, "臍帶拔脫照樣偵測離台");
  }
  {
    Sim s;
    toArmed(s);
    s.pad(FmPad::COUNTDOWN, 0, 0.1f);
    s.step();
    s.pad(FmPad::SAFE);  // 最後一刻中止
    s.step();
    CHECK(s.ds() == FmMode::PAD_IDLE && !s.c.state().fire_latched, "最後一幀後架 SAFE → PAD_IDLE、解閂");
  }
  {
    Sim s;
    toArmed(s);
    s.pad(FmPad::COUNTDOWN, 0, 0.1f);
    s.step();
    s.umbilicalGone();
    s.in.accel_u_mps2 = 0.0f;  // 根本沒點
    s.step(1200);
    CHECK(s.ds() == FmMode::HANGFIRE, "閂了但沒離台、臍帶也斷 → T+1 s HANGFIRE");
  }
}

// ---------------------------------------------------------------- 標稱剖面 FM-07→12
static void cpNominal() {
  std::printf("CP-NOMINAL（ARMED → 點火 → 助推 → 滑翔 → 4 s 分離 → 開傘 → 落地）\n");
  Sim s;
  toArmed(s);
  s.pad(FmPad::COUNTDOWN, 0, 1.0f);
  s.step(900);
  s.pad(FmPad::IGNITION, FmPad::FLAG_IGNITER_ON);
  s.step(21);  // 架 T+21 ms 鬆夾持
  s.pad(FmPad::RELEASED, FmPad::FLAG_IGNITER_ON | FmPad::FLAG_THRUST_CONFIRMED | FmPad::FLAG_CLAMP_RELEASED);
  float v = 0.0f;
  double tLift = -1, tActive = -1, tCoast = -1, tDescent = -1, tLanded = -1;
  for (int k = 0; k < 20000; ++k) {
    const bool burning = k < 1200;
    s.in.accel_u_mps2 = burning ? 30.0f : -9.8f;
    v += s.in.accel_u_mps2 * 1e-3f;
    s.in.v_est_mps = std::fabs(v);
    if (k == 60) s.umbilicalGone();  // 離軌拔臍帶
    if (s.tSep > 0 && s.t >= s.tSep + 0.030) s.in.sep_switch_open = true;  // 模擬：舵機放開 30 ms 後開關斷
    if (s.ds() == FmMode::LS_CHUTE && s.t >= s.tChuteDs + 5.0) s.in.contact_ds = true;  // 模擬：開傘 5 s 後著地
    const FmMode before = s.ds();
    s.step();
    if (before != s.ds()) {
      if (s.ds() == FmMode::LIFTOFF) tLift = s.t;
      if (s.ds() == FmMode::BOOST_ACTIVE) tActive = s.t;
      if (s.ds() == FmMode::COAST) tCoast = s.t;
      if (s.ds() == FmMode::LS_DESCENT) tDescent = s.t;
      if (s.ds() == FmMode::LS_LANDED) tLanded = s.t;
    }
  }
  CHECK(tLift > 0, "有 LIFTOFF");
  CHECK(tActive > 0 && tActive - tLift >= 0.1 - 2e-3, "BOOST_ACTIVE 不早於 t_passive_min");
  CHECK(tCoast > 0, "有 COAST");
  CHECK(s.nSep == 1, "分離恰一次");
  CHECK(s.lastMask == kFmSepMaskAll, "mask＝三顆 0x07");
  CHECK(std::fabs((s.tSep) - (tCoast - 1e-3) - 4.0) <= 2e-3, "進 COAST 後 4 s 分離");
  CHECK(tDescent > s.tSep, "開關斷 → LS_DESCENT");
  CHECK(s.nChuteDs == 1 && std::fabs(s.tChuteDs - s.tSep - 3.1) <= 2e-3, "分離下令後 3.1 s 開傘、只一次");
  CHECK(s.nSide == 0, "標稱不是側傘");
  CHECK(tLanded > 0 && s.ds() == FmMode::LS_LANDED, "著地 → LS_LANDED");
  CHECK(!s.c.state().us_chute_inhibit, "標稱不禁上級傘");
  CHECK(s.c.state().abort_done == false, "標稱不中止");
}

// ---------------------------------------------------------------- FM-12
static void cpInhibitSep() {
  std::printf("CP-INHIBIT-SEP\n");
  Sim s;
  FmState st;
  st.ds = FmMode::COAST;
  st.t_coast_s = 0.0;
  st.t_liftoff_s = -1.0;
  s.c.setStateForTest(st);
  s.in.sep_ok = true;
  s.in.accel_u_mps2 = -9.8f;
  s.step(800);  // t = 0 .. 0.799
  CHECK(s.nSep == 0 && s.ds() == FmMode::COAST, "抑制窗內不分離");
  s.step(2);
  CHECK(s.nSep == 1 && std::fabs(s.tSep - 0.8) <= 1.5e-3, "抑制窗滿（sep_ok）→ 分離（±1 拍）");
}

static void cpBackupSep() {
  std::printf("CP-BACKUP-SEP（4 s）\n");
  Sim s;
  FmState st;
  st.ds = FmMode::COAST;
  st.t_coast_s = 0.0;
  s.c.setStateForTest(st);
  s.in.accel_u_mps2 = -9.8f;
  s.step(10005);
  CHECK(s.nSep == 1 && std::fabs(s.tSep - 4.0) <= 2e-3, "COAST 後 4 s 分離");
}

static void cpNoEarlySep() {
  std::printf("CP-NO-EARLY-SEP\n");
  Sim s;
  FmState st;
  st.ds = FmMode::BOOST_ACTIVE;
  st.t_liftoff_s = 0.0;
  st.motor_on = true;
  s.c.setStateForTest(st);
  s.in.sep_ok = true;
  s.in.accel_u_mps2 = 20.0f;
  s.step(500);
  CHECK(s.nSep == 0, "BOOST_ACTIVE 不分離");
}

// ---------------------------------------------------------------- FM-06
static void cpAbortD12() {
  std::printf("CP-ABORT-D12\n");
  Sim s;
  FmState st;
  st.ds = FmMode::BOOST_ACTIVE;
  st.t_liftoff_s = 0.0;
  st.motor_on = true;
  s.c.setStateForTest(st);
  s.t = 0.20;
  s.in.tilt_rad = 31.0f * kDeg;
  s.in.accel_u_mps2 = -1.0f;
  s.step(1201);
  CHECK(s.ds() == FmMode::ABORT, "傾角 31° 持續 1 s → ABORT");
  CHECK(!s.c.state().motor_on, "中止 motor_on=false");
  CHECK(s.nSep == 1 && s.nChuteDs == 1, "中止：分離＋開傘各一次");
  CHECK(s.o.authority == FmAuthority::LOCKED_CENTER, "中止舵權 LOCKED_CENTER");
  CHECK(s.c.state().us_chute_inhibit, "中止禁上級傘");
}

static void cpAbortTipoff() {
  std::printf("CP-ABORT-TIPOFF（離台 200 ms 內不算）\n");
  Sim s;
  FmState st;
  st.ds = FmMode::BOOST_PASSIVE;
  st.t_liftoff_s = 0.0;
  s.c.setStateForTest(st);
  s.in.rate_rads = 200.0f * kDeg;
  s.step(199);
  CHECK(s.c.state().t_rate_s == 0.0f, "tip-off 窗內不累計");
}

static void cpWatchdog() {
  std::printf("CP-WATCHDOG（離台 20 s 仍在 COAST）\n");
  Sim s;
  FmState st;
  st.ds = FmMode::COAST;
  st.t_liftoff_s = 0.0;
  st.t_coast_s = 19.9;  // 4 s 分離還沒到
  s.c.setStateForTest(st);
  s.t = 19.95;
  s.step(60);
  CHECK(s.ds() == FmMode::ABORT, "看門狗 → ABORT");
}

// ---------------------------------------------------------------- FM-04
static void cpCmdWhitelist() {
  std::printf("CP-CMD-WHITELIST\n");
  Sim s;
  FmState st;
  st.ds = FmMode::BOOST_ACTIVE;
  st.t_liftoff_s = 0.0;
  s.c.setStateForTest(st);
  s.t = 1.0;
  s.cmd.override_ = true;
  s.step();
  CHECK(s.ds() == FmMode::BOOST_ACTIVE && s.n(FmEvent::REJECT_OVERRIDE) == 1, "飛行中 override 拒絕");
  s.cmd.abortValid = true;
  s.step();
  CHECK(s.ds() == FmMode::ABORT, "已驗證 ABORT → ABORT");
  s.cmd.abortValid = true;
  s.step();
  CHECK(s.n(FmEvent::REJECT_ABORT) == 1 && s.nSep == 1, "第二次 ABORT 拒絕、分離只一次");
  s.cmd.gimbal_ds = true;
  s.step();
  CHECK(s.n(FmEvent::DROP_GIMBAL_DS) == 1, "下級雲台丟棄");
}

static void cpPadAbortIgnored() {
  std::printf("CP-PAD-ABORT（墊上 ABORT 不受理，走架）\n");
  Sim s;
  toArmed(s);
  s.cmd.abortValid = true;
  s.step();
  CHECK(s.ds() == FmMode::ARMED && s.nSep == 0, "墊上 ABORT 不分離（中止由架處理）");
}

static void cpIllegalTrans() {
  std::printf("CP-ILLEGAL-TRANS\n");
  Sim s;
  s.cmd.requestMode = static_cast<int>(FmMode::SEPARATED);
  s.step();
  CHECK(s.ds() == FmMode::PAD_IDLE && s.n(FmEvent::REJECT_TRANS) == 1, "非法 request_mode 拒絕");
  s.cmd.requestMode = static_cast<int>(FmMode::ARMED);
  s.step();
  CHECK(s.ds() == FmMode::PAD_IDLE && s.n(FmEvent::REJECT_TRANS) == 1, "合法 request_mode 也不切");
}

// ---------------------------------------------------------------- FM-14／16
static void cpChuteDs() {
  std::printf("CP-CHUTE-DS（FM-14／16 邏輯）\n");
  {
    Sim s;
    FmState st;
    st.ds = FmMode::LS_DESCENT;
    st.t_sep_cmd_s = 0.0;
    st.sep_fired = true;
    s.c.setStateForTest(st);
    s.in.accel_u_mps2 = 10.0f;  // accel_high
    s.step(3500);
    CHECK(s.nChuteDs == 0, "accel_high 抑制開傘");
    s.in.accel_u_mps2 = -9.8f;
    s.step();
    CHECK(s.nChuteDs == 1 && s.ds() == FmMode::LS_CHUTE, "加速度降下 → 開傘、LS_CHUTE");
  }
  {
    Sim s;
    FmState st;
    st.ds = FmMode::SEP_FAIL;
    st.t_sep_cmd_s = 0.0;
    st.sep_fired = true;
    s.c.setStateForTest(st);
    s.step(3200);
    CHECK(s.nChuteDs == 1 && s.nSide == 1 && std::fabs(s.tChuteDs - 3.1) <= 2e-3, "SEP_FAIL 3.1 s 側傘");
    CHECK(s.ds() == FmMode::SEP_FAIL && s.n(FmEvent::SIDE_CHUTE) == 1, "模式留在 SEP_FAIL");
  }
}

// ---------------------------------------------------------------- FM-02 舵權
static void cpImuStale() {
  std::printf("CP-IMU-STALE\n");
  Sim s;
  FmState st;
  st.ds = FmMode::BOOST_ACTIVE;
  st.t_liftoff_s = 0.0;
  s.c.setStateForTest(st);
  s.t = 0.1;
  s.step();
  CHECK(s.o.authority == FmAuthority::CLOSED_LOOP, "BOOST_ACTIVE 閉環");
  s.in.imu_fresh = false;
  s.step(21);  // 20 ms（float 累加差一拍以內）
  CHECK(s.o.authority == FmAuthority::CENTER, "IMU 停 20 ms → CENTER");
  s.in.imu_fresh = true;
  s.step();
  CHECK(s.o.authority == FmAuthority::CLOSED_LOOP, "IMU 恢復 → 閉環");
}

// ---------------------------------------------------------------- 墊上安全
static void cpPadSafe() {
  std::printf("CP-PAD-SAFE（沒有架狀態：最壞輸入 60 s 不離開 PAD_IDLE）\n");
  Sim s;
  s.in.sep_switch_open = s.in.sep_ok = s.in.contact_ds = s.in.contact_us = true;
  s.in.accel_u_mps2 = 100.0f;
  s.in.tilt_rad = 3.0f;
  s.in.rate_rads = 50.0f;
  for (int k = 0; k < 60000; ++k) {
    s.cmd.abortValid = s.cmd.override_ = true;
    s.step();
  }
  CHECK(s.ds() == FmMode::PAD_IDLE && s.nSep == 0 && s.nChuteDs == 0, "墊上不轉移、不分離、不開傘");
}


// ---------------------------------------------------------------- FM-13：分離失敗
static void cpSepFail() {
  std::printf("CP-SEP-FAIL（下令後開關一直沒斷）\n");
  Sim s;
  FmState st;
  st.ds = FmMode::COAST;
  st.t_coast_s = 0.0;
  st.t_liftoff_s = -1.0;
  s.c.setStateForTest(st);
  s.in.accel_u_mps2 = -9.8f;
  s.step(10000);  // 4 s 分離＋5 s 逾時＋裕度
  CHECK(s.nSep == 1, "分離下令一次");
  CHECK(s.ds() == FmMode::SEP_FAIL && s.n(FmEvent::SEP_FAIL) == 1, "5 s 開關仍通 → SEP_FAIL");
  CHECK(s.nChuteDs == 1 && s.nSide == 1, "SEP_FAIL 開側傘");
  CHECK(std::fabs(s.tChuteDs - (s.tSep + 5.0)) <= 2e-3, "側傘在 SEP_FAIL 當拍（已過 3.1 s）");
  CHECK(s.c.state().us_chute_inhibit, "禁上級傘");
}

// ---------------------------------------------------------------- FM-05：假分
static void cpFalseSep() {
  std::printf("CP-FALSE-SEP（還沒下令開關就斷）\n");
  Sim s;
  FmState st;
  st.ds = FmMode::BOOST_ACTIVE;
  st.t_liftoff_s = 0.0;
  s.c.setStateForTest(st);
  s.t = 0.5;
  s.in.accel_u_mps2 = 20.0f;
  s.in.sep_switch_open = true;
  s.step();
  CHECK(s.c.state().us_chute_inhibit, "假分 → 禁上級傘");
  CHECK(s.nSep == 0, "假分不觸發分離");
}

// ---------------------------------------------------------------- FM-17：中止後續
static void cpAbortFollowUp() {
  std::printf("CP-ABORT-FOLLOW（中止後分得開／分不開）\n");
  {
    Sim s;
    FmState st;
    st.ds = FmMode::BOOST_ACTIVE;
    st.t_liftoff_s = 0.0;
    s.c.setStateForTest(st);
    s.t = 1.0;
    s.cmd.abortValid = true;
    s.step(30);
    s.in.sep_switch_open = true;
    s.step();
    CHECK(s.ds() == FmMode::LS_DESCENT, "中止後開關斷 → LS_DESCENT");
    s.step(5000);
    CHECK(s.nChuteDs == 1, "中止已開過傘，不再開第二次");
  }
  {
    Sim s;
    FmState st;
    st.ds = FmMode::BOOST_ACTIVE;
    st.t_liftoff_s = 0.0;
    s.c.setStateForTest(st);
    s.t = 1.0;
    s.cmd.abortValid = true;
    s.step(5100);
    CHECK(s.ds() == FmMode::SEP_FAIL && s.n(FmEvent::SEP_FAIL) == 1, "中止後 5 s 開關仍通 → SEP_FAIL");
  }
}

// ---------------------------------------------------------------- FM-15：著陸備援
static void cpLandMax() {
  std::printf("CP-LAND-MAX（沒收到著地，離台 50 s 備援）\n");
  Sim s;
  FmState st;
  st.ds = FmMode::LS_CHUTE;
  st.t_liftoff_s = 0.0;
  s.c.setStateForTest(st);
  s.t = 49.9;
  s.step(99);
  CHECK(s.ds() == FmMode::LS_CHUTE, "50 s 前不落地");
  s.step(2);
  CHECK(s.ds() == FmMode::LS_LANDED, "50 s → LS_LANDED");
}

// ---------------------------------------------------------------- FM-18：上級
static void cpUs() {
  std::printf("CP-US（上級：開關斷 → 3.2 s 開傘 → 著地）\n");
  Sim s(FmRole::US);
  s.step(1000);
  CHECK(s.o.mode == FmMode::US_WAIT, "開關沒斷 → US_WAIT");
  s.in.sep_switch_open = true;
  const double tOpen = s.t;
  s.step();
  CHECK(s.o.mode == FmMode::US_COAST, "開關斷 → US_COAST");
  s.step(3300);
  CHECK(s.nChuteUs == 1 && s.o.mode == FmMode::US_CHUTE, "3.2 s 開上級傘、只一次");
  CHECK(s.n(FmEvent::CMD_CHUTE_US) == 1, "記 CMD_CHUTE_US");
  (void)tOpen;
  s.in.contact_us = true;
  s.step();
  CHECK(s.o.mode == FmMode::US_LANDED, "著地 → US_LANDED");
}

int main() {
  cpInterlock();
  cpHangfire();
  cpFinalCountdown();
  cpNominal();
  cpInhibitSep();
  cpBackupSep();
  cpNoEarlySep();
  cpAbortD12();
  cpAbortTipoff();
  cpWatchdog();
  cpCmdWhitelist();
  cpPadAbortIgnored();
  cpIllegalTrans();
  cpChuteDs();
  cpImuStale();
  cpPadSafe();
  cpSepFail();
  cpFalseSep();
  cpAbortFollowUp();
  cpLandMax();
  cpUs();
  if (g_fail) {
    std::printf("FlightModeCore: FAIL (%d)\n", g_fail);
    return 1;
  }
  std::printf("FlightModeCore: PASS（21 組）\n");
  return 0;
}
