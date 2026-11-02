// ServoSimCore／SepSimCore host 測試（不需 F´）：見 ../run_host_tests.sh
#include "../ServoSimCore.hpp"
#include <cstdio>
#include <vector>
using namespace ServoSim;
static int g_fail = 0, g_cases = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct Hw : ISimHw {
  int passAfter = 3; bool failNow = false; int n = 0;
  bool selfTestDone(bool& fail) override { fail = failNow; return !failNow && ++n >= passAfter; }
};

struct SSink : IServoSink {
  uint16_t pwm[4] = {0, 0, 0, 0}; int ready = 0, fault = 0, timeout = 0, lockCmd = 0, lockTo = 0, uv = 0;
  std::vector<DropReason> drops; std::vector<uint16_t> clampUsed; FaultReason lastFault{};
  void pwmOut(uint8_t ch, uint16_t us) override { pwm[ch] = us; }
  void evEnterReady() override { ready++; }
  void evEnterFault(FaultReason r) override { fault++; lastFault = r; }
  void evCmdDropped(uint32_t, DropReason r) override { drops.push_back(r); }
  void evPulseClamped(uint8_t, uint16_t, uint16_t u) override { clampUsed.push_back(u); }
  void evCmdTimeout(float) override { timeout++; }
  void evLockNeutral(NeutralSource s) override { (s == NeutralSource::CMD ? lockCmd : lockTo)++; }
  void evUvSeen(UvState) override { uv++; }
};

struct SRig {
  ServoParams p; Hw hw; SSink sink; ServoSimCore core;
  explicit SRig(uint8_t chans = 4) : p(mk(chans)), core(p, hw, sink) {}
  static ServoParams mk(uint8_t c) { ServoParams p; p.channel_count = c; return p; }
  void run(int n) { for (int i = 0; i < n; i++) core.tick(); }
  void boot() { core.start(); run(5); }
  void cmd(uint32_t seq, uint16_t a, uint16_t b = 1500, uint16_t c = 1500, uint16_t d = 1500, bool lock = false) {
    ServoCmd k{0.0, seq, {a, b, c, d}, lock}; core.cmdIn(k);
  }
};

static void tServoHandshake() {
  g_cases++; SRig r;
  CHECK(r.core.drvState() == DrvState::OFF);
  r.core.tick(); CHECK(r.core.drvState() == DrvState::OFF);
  r.core.start(); CHECK(r.core.drvState() == DrvState::INIT);
  r.core.tick(); CHECK(r.core.drvState() == DrvState::SELFTEST);
  r.run(3); CHECK(r.core.drvState() == DrvState::READY); CHECK(r.sink.ready == 1);
  CHECK(r.sink.pwm[0] == 1500 && r.sink.pwm[3] == 1500);
}
static void tServoFault() {
  g_cases++; { SRig r; r.hw.failNow = true; r.boot(); CHECK(r.core.drvState() == DrvState::FAULT); CHECK(r.sink.lastFault == FaultReason::SELFTEST_FAIL); }
  g_cases++; { SRig r; r.hw.passAfter = 1000; r.core.start(); r.run(50); CHECK(r.core.drvState() == DrvState::SELFTEST); r.run(1);  // 1 tick INIT + 50 tick (1.0 s) 截止
    CHECK(r.core.drvState() == DrvState::FAULT); CHECK(r.sink.lastFault == FaultReason::SELFTEST_TIMEOUT);
    r.cmd(1, 1800); r.run(1); CHECK(r.sink.pwm[0] == 1500); CHECK(r.sink.drops.back() == DropReason::NOT_READY); }
}
static void tServoClamp() {
  g_cases++; SRig r; r.boot();
  r.cmd(1, 900, 2100, 1700, 1000); r.run(1);
  CHECK(r.sink.pwm[0] == 1000 && r.sink.pwm[1] == 2000 && r.sink.pwm[2] == 1700 && r.sink.pwm[3] == 1000);
  CHECK(r.sink.clampUsed.size() == 2 && r.core.clampCount() == 2);
}
static void tServoSeq() {
  g_cases++; SRig r; r.boot();
  r.cmd(10, 1600); r.cmd(10, 1900); r.cmd(9, 1900); r.run(1);
  CHECK(r.sink.pwm[0] == 1600); CHECK(r.core.dropCount() == 2);
  CHECK(r.sink.drops[0] == DropReason::SEQ_DUP && r.sink.drops[1] == DropReason::SEQ_BACKWARD);
  r.cmd(11, 1700); r.run(1); CHECK(r.sink.pwm[0] == 1700 && r.core.lastSeq() == 11);
}
static void tServoLockNeutral() {
  g_cases++; SRig r; r.boot();
  r.cmd(1, 1800, 1200); r.run(1); CHECK(r.sink.pwm[0] == 1800);
  r.cmd(2, 1800, 1200, 1800, 1800, true); r.run(1);
  CHECK(r.sink.pwm[0] == 1500 && r.sink.pwm[1] == 1500 && r.sink.lockCmd == 1);
}
static void tServoTimeout() {
  g_cases++; SRig r; r.boot();
  r.cmd(1, 1800); r.run(2);  // 0.04 s ≤ 0.05：保持
  CHECK(r.sink.pwm[0] == 1800 && r.sink.timeout == 0);
  r.run(1);  // 0.06 s > 0.05：回中立
  CHECK(r.sink.pwm[0] == 1500 && r.sink.timeout == 1 && r.sink.lockTo == 1);
  r.run(10); CHECK(r.sink.timeout == 1);  // 只發一次
  r.cmd(2, 1300); r.run(1); CHECK(r.sink.pwm[0] == 1300);  // 新命令恢復
}
static void tServoUv() {
  g_cases++; SRig r; r.boot(); r.cmd(1, 1800); r.run(1);
  r.core.uvIn(UvState::UV_CRITICAL); r.run(1);
  CHECK(r.sink.pwm[0] == 1800 && r.sink.uv == 1 && r.core.uvState() == UvState::UV_CRITICAL);
}
static void tGimbalTwoCh() {
  g_cases++; SRig r(2); r.boot();
  r.cmd(1, 1600, 1400, 900, 2500); r.run(1);
  CHECK(r.sink.pwm[0] == 1600 && r.sink.pwm[1] == 1400);
  CHECK(r.sink.pwm[2] == 0 && r.sink.pwm[3] == 0);  // [2]、[3] 不輸出
  CHECK(r.sink.clampUsed.empty());                    // 也不夾限、不發事件
}

// ---------------- SEP ----------------
struct Nv : INvFlags {
  uint8_t mask = 0; bool failSet = false, failGet = false; int sets = 0;
  bool sepReleasedGet(uint8_t& m) override { m = mask; return !failGet; }
  bool sepReleasedSet(uint8_t m) override { sets++; if (failSet) return false; mask |= m; return true; }  // 只能 set
};
struct PSink : ISepSink {
  uint16_t pwm[4] = {0, 0, 0, 0}; int ready = 0, fault = 0, uv = 0;
  std::vector<uint8_t> fire, already, nvFail, holdMask; std::vector<DropReason> drops; FaultReason lastFault{};
  void pwmOut(uint8_t ch, uint16_t us) override { pwm[ch] = us; }
  void evEnterReady() override { ready++; }
  void evEnterFault(FaultReason r) override { fault++; lastFault = r; }
  void evCmdDropped(uint32_t, DropReason r) override { drops.push_back(r); }
  void evSepFire(uint8_t c) override { fire.push_back(c); }
  void evSepAlreadyFired(uint8_t c) override { already.push_back(c); }
  void evSepFaultHold(uint8_t m) override { holdMask.push_back(m); }
  void evSepNvWriteFail(uint8_t c) override { nvFail.push_back(c); }
  void evUvSeen(UvState) override { uv++; }
};
struct PRig {
  SepParams p; Hw hw; Nv nv; PSink sink; SepSimCore core{p, hw, nv, sink};
  void run(int n) { for (int i = 0; i < n; i++) core.tick(); }
  void boot() { core.start(); run(5); }
  void cmd(uint32_t seq, uint8_t mask, bool valid = true) { SepCmd k{0.0, seq, mask, valid}; core.cmdIn(k); }
};

static void tSepHandshake() {
  g_cases++; { PRig r; r.boot(); CHECK(r.core.drvState() == DrvState::READY); CHECK(r.sink.pwm[0] == 1000 && r.sink.pwm[3] == 1000); }
  g_cases++; { PRig r; r.hw.failNow = true; r.boot(); CHECK(r.core.drvState() == DrvState::FAULT);
    CHECK(r.sink.holdMask.size() == 1 && r.sink.holdMask[0] == 0x0F); CHECK(r.sink.pwm[0] == 1000); }
  g_cases++; { PRig r; r.nv.failGet = true; r.boot(); CHECK(r.core.drvState() == DrvState::FAULT && r.sink.lastFault == FaultReason::NV_READ_FAIL); }
}
static void tSepFireOnce() {
  g_cases++; PRig r; r.boot();
  r.cmd(1, 0x05); r.run(1);  // SEP_1、SEP_3
  CHECK(r.sink.fire.size() == 2 && r.sink.pwm[0] == 2000 && r.sink.pwm[2] == 2000 && r.sink.pwm[1] == 1000);
  CHECK(r.nv.mask == 0x05 && r.core.firedMask() == 0x05 && r.core.lockedMask() == 0x0A);
  CHECK(r.core.activeMask() == 0x05);
  r.run(25); CHECK(r.sink.pwm[0] == 2000 && r.core.activeMask() == 0);  // fire_hold_s 後仍停在 fire_us
  r.run(100); CHECK(r.sink.pwm[0] == 2000 && r.sink.pwm[2] == 2000);
  r.cmd(2, 0x01); r.run(1);
  CHECK(r.sink.already.size() == 1 && r.sink.already[0] == 0 && r.sink.pwm[0] == 2000 && r.nv.sets == 2);
}
static void tSepNvPersist() {  // 重開機後旗標仍在
  g_cases++; PRig r; r.nv.mask = 0x02; r.boot();
  CHECK(r.core.firedMask() == 0x02 && r.sink.pwm[1] == 1000);  // 開機已釋放：輸出 lock_us，不重發 fire_us
  r.cmd(1, 0x02); r.run(1);
  CHECK(r.sink.already.size() == 1 && r.sink.fire.empty() && r.nv.sets == 0);
}
static void tSepNvWriteFail() {
  g_cases++; PRig r; r.boot(); r.nv.failSet = true;
  r.cmd(1, 0x01); r.run(1);
  CHECK(r.sink.nvFail.size() == 1 && r.sink.fire.empty() && r.sink.pwm[0] == 1000);
  CHECK(r.core.drvState() == DrvState::READY && r.sink.fault == 0);  // 不進 FAULT
  r.nv.failSet = false; r.cmd(2, 0x01); r.run(1);  // 重試
  CHECK(r.sink.fire.size() == 1 && r.sink.pwm[0] == 2000);
}
static void tSepSeqAndInvalid() {
  g_cases++; PRig r;
  r.cmd(1, 0x01); CHECK(r.sink.drops.back() == DropReason::NOT_READY);
  r.boot();
  r.cmd(5, 0x01, false); CHECK(r.sink.drops.back() == DropReason::INVALID && r.sink.fire.empty());
  r.cmd(5, 0x01); r.cmd(5, 0x02); r.cmd(4, 0x04); r.run(1);
  CHECK(r.sink.fire.size() == 1 && r.sink.pwm[1] == 1000 && r.sink.pwm[2] == 1000);
}
static void tSepNoTimeoutAndUv() {
  g_cases++; PRig r; r.boot(); r.cmd(1, 0x08); r.run(5);
  r.core.uvIn(UvState::UV_SHED); r.run(200);  // 長時間無命令：SEP 不動
  CHECK(r.sink.pwm[3] == 2000 && r.sink.pwm[0] == 1000 && r.sink.uv == 1 && r.core.firedMask() == 0x08);
}

int main() {
  tServoHandshake(); tServoFault(); tServoClamp(); tServoSeq(); tServoLockNeutral(); tServoTimeout(); tServoUv(); tGimbalTwoCh();
  tSepHandshake(); tSepFireOnce(); tSepNvPersist(); tSepNvWriteFail(); tSepSeqAndInvalid(); tSepNoTimeoutAndUv();
  std::printf("%s: %d cases, %d failures\n", g_fail ? "FAILED" : "PASSED", g_cases, g_fail);
  return g_fail ? 1 : 0;
}
