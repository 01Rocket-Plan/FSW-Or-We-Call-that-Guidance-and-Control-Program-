// PadLinkHostTest：Issue #24 箭↔架（臍帶）／架↔地面（NRF24）／架上主控 host 自測
// 不需要 F´。金鑰：只用明顯假的測試驗證器，不是密碼學、不能上飛行。
#include "PadCmdGate.hpp"
#include "PadControllerCore.hpp"
#include "PadLinkCodec.hpp"
#include "UmbilicalTxCore.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

using namespace PadLink;

namespace {

int g_fail = 0;
#define CHECK(c)                                                         \
  do {                                                                   \
    if (!(c)) {                                                          \
      std::printf("CHECK failed: %s (%s:%d)\n", #c, __FILE__, __LINE__); \
      ++g_fail;                                                          \
    }                                                                    \
  } while (0)

// FAKE_TEST_ONLY：簡單的鍵控雜湊，只用來測試流程；飛行要接 HMAC-SHA256（PR #13）。
const std::uint8_t kFakeKey[] = "FAKE_PAD_TEST_KEY_DO_NOT_FLY";
void fakeTag(const std::uint8_t* msg, std::size_t n, std::uint8_t out[kCmdTagBytes]) {
  std::uint32_t h = 2166136261u;
  for (std::size_t i = 0; i < sizeof(kFakeKey); ++i) h = (h ^ kFakeKey[i]) * 16777619u;
  for (std::size_t i = 0; i < n; ++i) h = (h ^ msg[i]) * 16777619u;
  for (std::size_t i = 0; i < kCmdTagBytes; ++i) {
    h = (h ^ static_cast<std::uint32_t>(i)) * 16777619u;
    out[i] = static_cast<std::uint8_t>(h >> 24);
  }
}
struct FakeAuth : IPadCmdAuth {
  bool verify(const std::uint8_t* msg, std::size_t n, const std::uint8_t* tag) override {
    std::uint8_t want[kCmdTagBytes];
    fakeTag(msg, n, want);
    std::uint8_t diff = 0;
    for (std::size_t i = 0; i < kCmdTagBytes; ++i) diff |= static_cast<std::uint8_t>(want[i] ^ tag[i]);
    return diff == 0;
  }
};

std::vector<std::uint8_t> cmdFrame(PadCmdOp op, std::uint32_t counter, std::uint8_t bootId,
                                   bool goodTag = true) {
  GroundCmdMsg c;
  c.op = op;
  c.counter = counter;
  c.bootId = bootId;
  std::uint8_t mac[PadCmdGate::kMacMsgBytes];
  const std::size_t n = PadCmdGate::buildMacMessage(bootId, counter, op, mac, sizeof(mac));
  fakeTag(mac, n, c.tag);
  if (!goodTag) c.tag[0] ^= 0x01u;
  std::uint8_t p[kGroundCmdBytes];
  packGroundCmd(c, p, sizeof(p));
  std::vector<std::uint8_t> f(kMaxFrameBytes);
  f.resize(encodeFrame(PadMsgType::GROUND_CMD, 0, p, sizeof(p), f.data(), f.size()));
  return f;
}

// 臍帶回送：UmbilicalTxCore 寫出的位元組直接餵 PadControllerCore。
struct LoopUart : IUmbilicalUart {
  PadControllerCore* pad = nullptr;
  double now = 0.0;
  bool connected = true;
  int write(const std::uint8_t* d, std::size_t n) override {
    if (!connected) return 0;
    if (pad != nullptr) pad->onUmbilicalBytes(d, n, now);
    return static_cast<int>(n);
  }
};

constexpr std::uint8_t kBoot = 7;
constexpr std::uint16_t kBitAllOk = kRocketBitHardMask | BIT_SD_OK | BIT_TLM_OK;

PadInputs greenInputs() {
  PadInputs in;
  in.seated = true;
  in.continuity = true;
  in.keyOn = true;
  in.railJam = false;
  in.tilt_rad = 0.01;
  in.padV_V = 12.4;
  return in;
}

struct EvRec : IPadEvents {
  PadReason lastCmdReason = PadReason::NONE;
  int stateChanges = 0;
  void onStateChange(PadState, PadState, PadReason) override { ++stateChanges; }
  void onCommand(PadCmdOp, PadCmdResult, PadReason why) override { lastCmdReason = why; }
};

//! 一套「架＋箭＋地面」模擬：100 Hz 跑架，10 Hz 送臍帶，5 Hz 送地面心跳。
struct Rig {
  FakeAuth auth;
  EvRec ev;
  PadControllerCore pad{&auth, kBoot, &ev};
  LoopUart uart;
  UmbilicalTxCore rocket{uart, 3};
  PadInputs in = greenInputs();
  PadOutputs out;
  double t = 0.0;
  std::uint32_t ctr = 0;
  bool groundAlive = true;
  bool everIgniterWithoutKey = false;
  bool clampReleasedThenLocked = false;
  bool sawClampRelease = false;

  Rig() {
    uart.pad = &pad;
    rocket.setMode(kModePadIdle);
    rocket.setBattV(15.8);
  }
  PadCmdResult send(PadCmdOp op, bool goodTag = true) {
    const auto f = cmdFrame(op, ++ctr, kBoot, goodTag);
    return pad.onGroundPacket(f.data(), f.size(), t);
  }
  void step(double dt = 0.01) {
    t += dt;
    uart.now = t;
    rocket.setBit(kBitAllOk, t);
    rocket.tick(t);
    if (groundAlive && static_cast<long>(t * 100.0 + 0.5) % 20 == 0) send(PadCmdOp::HEARTBEAT);
    out = pad.tick(in, t);
    if (out.igniterOn && !in.keyOn) everIgniterWithoutKey = true;
    if (sawClampRelease && !out.clampRelease) clampReleasedThenLocked = true;
    if (out.clampRelease) sawClampRelease = true;
  }
  void run(double sec) {
    const int n = static_cast<int>(sec * 100.0 + 0.5);
    for (int i = 0; i < n; ++i) step();
  }
  void warmUp() { run(1.0); }
};

// ---------------------------------------------------------------------------

void testCodecRoundTripAndCrc() {
  RocketStatusMsg r;
  r.flightMode = 1;
  r.bitFlags = kBitAllOk;
  r.ready = true;
  r.battV_mV = 15800;
  r.bootCount = 9;
  std::uint8_t p[kRocketStatusBytes];
  CHECK(packRocketStatus(r, p, sizeof(p)) == kRocketStatusBytes);
  std::uint8_t f[kMaxFrameBytes];
  const std::size_t n = encodeFrame(PadMsgType::ROCKET_STATUS, 0x1234, p, sizeof(p), f, sizeof(f));
  CHECK(n == kOverheadBytes + kRocketStatusBytes);
  DecodedFrame d;
  CHECK(decodeFrame(f, n, d));
  CHECK(d.seq == 0x1234 && d.type == PadMsgType::ROCKET_STATUS);
  RocketStatusMsg r2;
  CHECK(unpackRocketStatus(d.payload, d.len, r2));
  CHECK(r2.ready && r2.battV_mV == 15800 && r2.bitFlags == kBitAllOk && r2.bootCount == 9);
  f[7] ^= 0x40u;  // 翻一個位元
  CHECK(!decodeFrame(f, n, d));
  // CRC-16/CCITT-FALSE 標準測試向量 "123456789" → 0x29B1
  CHECK(crc16Ccitt(reinterpret_cast<const std::uint8_t*>("123456789"), 9) == 0x29B1u);
  // 酬載過長拒絕
  std::uint8_t big[kMaxPayload + 1] = {};
  CHECK(encodeFrame(PadMsgType::PAD_STATUS, 0, big, sizeof(big), f, sizeof(f)) == 0);
}

void testEveryMessageFitsNrf24() {
  CHECK(kMaxFrameBytes == kNrf24PayloadBytes);
  CHECK(kOverheadBytes + kPadStatusBytes <= kNrf24PayloadBytes);
  CHECK(kOverheadBytes + kGroundCmdBytes <= kNrf24PayloadBytes);
  PadControllerCore pad(nullptr, 1);
  std::uint8_t f[kMaxFrameBytes];
  CHECK(pad.buildStatusFrame(f, sizeof(f)) <= kNrf24PayloadBytes);
  CHECK(cmdFrame(PadCmdOp::ARM, 1, 1).size() <= kNrf24PayloadBytes);
}

void testStreamDeframerResync() {
  RocketStatusMsg r;
  r.ready = true;
  std::uint8_t p[kRocketStatusBytes];
  packRocketStatus(r, p, sizeof(p));
  std::uint8_t good[kMaxFrameBytes];
  const std::size_t n = encodeFrame(PadMsgType::ROCKET_STATUS, 5, p, sizeof(p), good, sizeof(good));
  std::vector<std::uint8_t> s = {0x00, 0xA5, 0xA5, 0x13, 0xFF};  // 雜訊＋假 sync
  std::vector<std::uint8_t> bad(good, good + n);
  bad[8] ^= 0x01u;  // 壞 CRC 的一幀
  s.insert(s.end(), bad.begin(), bad.end());
  s.insert(s.end(), good, good + n);
  StreamDeframer df;
  DecodedFrame d;
  int got = 0;
  for (std::uint8_t b : s) {
    if (df.push(b, d)) ++got;
  }
  CHECK(got == 1);
  CHECK(d.seq == 5);
  CHECK(df.crcErrors() == 1);
}

void testCmdGate() {
  FakeAuth auth;
  PadCmdGate g(&auth, kBoot);
  auto chk = [&](PadCmdOp op, std::uint32_t c, std::uint8_t boot, bool good) {
    const auto f = cmdFrame(op, c, boot, good);
    DecodedFrame d;
    GroundCmdMsg m;
    decodeFrame(f.data(), f.size(), d);
    unpackGroundCmd(d.payload, d.len, m);
    return g.check(m);
  };
  CHECK(chk(PadCmdOp::ARM, 1, kBoot, false) == PadCmdResult::AUTH_FAIL);
  CHECK(chk(PadCmdOp::ARM, 1, kBoot, true) == PadCmdResult::ACCEPTED);
  CHECK(chk(PadCmdOp::ARM, 1, kBoot, true) == PadCmdResult::REPLAY);
  CHECK(chk(PadCmdOp::LAUNCH, 2, kBoot + 1, true) == PadCmdResult::WRONG_BOOT);  // 上次開機的封包
  CHECK(chk(PadCmdOp::ABORT, 0, 0, false) == PadCmdResult::ACCEPTED);  // 安全方向不驗簽名
  CHECK(g.lastCounter() == 1u);                                       // ABORT 不推進計數器
  CHECK(chk(PadCmdOp::HEARTBEAT, 2, kBoot, false) == PadCmdResult::AUTH_FAIL);
  PadCmdGate none(nullptr, kBoot);  // 沒接驗證器 → deny-by-default
  GroundCmdMsg m;
  m.op = PadCmdOp::ARM;
  m.counter = 1;
  m.bootId = kBoot;
  CHECK(none.check(m) == PadCmdResult::NO_VERIFIER);
}

void testNominalLaunch() {
  Rig r;
  r.warmUp();
  CHECK(r.pad.state() == PadState::SAFE);
  CHECK((r.pad.interlocks() & kPadPermitMask) == kPadPermitMask);
  CHECK(!r.out.igniterOn && !r.out.clampRelease);
  CHECK(r.send(PadCmdOp::ARM) == PadCmdResult::ACCEPTED);
  CHECK(r.pad.state() == PadState::ARMED);
  CHECK(r.send(PadCmdOp::LAUNCH) == PadCmdResult::ACCEPTED);
  CHECK(r.pad.state() == PadState::COUNTDOWN);
  r.run(29.5);
  CHECK(r.pad.state() == PadState::COUNTDOWN);
  CHECK(!r.out.igniterOn);
  r.run(0.6);
  CHECK(r.pad.state() == PadState::IGNITION);
  CHECK(r.out.igniterOn && !r.out.clampRelease);  // 先點、夾持還鎖著
  r.in.thrustOk = true;
  r.run(0.05);
  CHECK(r.pad.state() == PadState::IGNITION);  // 還沒持續滿 thrustConfirm_s
  r.run(0.1);
  CHECK(r.pad.state() == PadState::RELEASED);
  CHECK(!r.out.igniterOn && r.out.clampRelease);
  // 起飛：臍帶拔脫、掛耳離軌；架上不應報錯
  r.uart.connected = false;
  r.in.offRail = true;
  r.in.seated = false;
  r.step();
  CHECK(r.pad.state() == PadState::LIFTED_OFF);
  CHECK(r.pad.lastReason() == PadReason::NONE);
  r.run(2.0);
  CHECK(r.pad.state() == PadState::LIFTED_OFF);
  CHECK(r.out.clampRelease && !r.out.igniterOn);
  CHECK(!r.clampReleasedThenLocked);
  CHECK(!r.everIgniterWithoutKey);
  // 一次開機只點一次
  CHECK(r.send(PadCmdOp::ARM) == PadCmdResult::REJECTED);
}

void testArmRejectedWhenAnyRed() {
  struct Case {
    void (*apply)(Rig&);
    PadReason want;
  } cases[] = {
      {[](Rig& r) { r.in.seated = false; }, PadReason::NOT_SEATED},
      {[](Rig& r) { r.in.continuity = false; }, PadReason::NO_CONTINUITY},
      {[](Rig& r) { r.in.keyOn = false; }, PadReason::KEY_OFF},
      {[](Rig& r) { r.in.tilt_rad = 0.2; }, PadReason::TILT},
      {[](Rig& r) { r.in.railJam = true; }, PadReason::RAIL_JAM},
      {[](Rig& r) { r.in.padV_V = 9.0; }, PadReason::PAD_POWER},
      {[](Rig& r) { r.uart.connected = false; }, PadReason::ROCKET_LINK_LOST},
  };
  for (const Case& c : cases) {
    Rig r;
    r.warmUp();
    c.apply(r);
    r.run(1.0);
    CHECK(r.send(PadCmdOp::ARM) == PadCmdResult::REJECTED);
    CHECK(r.pad.state() == PadState::SAFE);
    CHECK(r.pad.lastReason() == c.want);
  }
  // 箭體沒 READY（硬擋 BIT 沒過）
  Rig r;
  r.warmUp();
  for (int i = 0; i < 100; ++i) {
    r.t += 0.01;
    r.uart.now = r.t;
    r.rocket.setBit(kBitAllOk & static_cast<std::uint16_t>(~BIT_IMU_OK), r.t);
    r.rocket.tick(r.t);
    if (i % 20 == 0) r.send(PadCmdOp::HEARTBEAT);
    r.pad.tick(r.in, r.t);
  }
  CHECK(r.send(PadCmdOp::ARM) == PadCmdResult::REJECTED);
  CHECK(r.pad.lastReason() == PadReason::ROCKET_NOT_READY);
}

void testNoGroundLinkNoArm() {
  Rig r;
  r.groundAlive = false;
  r.run(1.0);
  // 第一包簽名 ARM 本身會讓地面鏈路變綠，所以要測「只有偽造心跳」的情況：
  for (int i = 0; i < 5; ++i) {
    const auto f = cmdFrame(PadCmdOp::HEARTBEAT, 1000 + i, kBoot, false);
    r.pad.onGroundPacket(f.data(), f.size(), r.t);
  }
  r.step();
  CHECK(!(r.pad.interlocks() & IL_GROUND_LINK_OK));  // 偽造心跳不算鏈路
  CHECK(r.send(PadCmdOp::ARM, false) == PadCmdResult::AUTH_FAIL);
  CHECK(r.pad.state() == PadState::SAFE);
}

void testLaunchNeedsArmFirst() {
  Rig r;
  r.warmUp();
  CHECK(r.send(PadCmdOp::LAUNCH) == PadCmdResult::REJECTED);
  CHECK(r.pad.state() == PadState::SAFE);
}

void testRedDuringCountdownNeedsFullRearm() {
  Rig r;
  r.warmUp();
  r.send(PadCmdOp::ARM);
  r.send(PadCmdOp::LAUNCH);
  r.run(10.0);
  r.in.seated = false;  // 間歇 seated=false
  r.step();
  CHECK(r.pad.state() == PadState::SAFE);
  CHECK(r.pad.lastReason() == PadReason::NOT_SEATED);
  r.in.seated = true;
  r.run(1.0);
  CHECK(r.pad.state() == PadState::SAFE);  // 不會自動恢復
  CHECK(r.send(PadCmdOp::LAUNCH) == PadCmdResult::REJECTED);  // 要重走 ARM → LAUNCH
  CHECK(r.send(PadCmdOp::ARM) == PadCmdResult::ACCEPTED);
  CHECK(!r.pad.ignitionUsed());
}

void testGroundLossDuringCountdownAborts() {
  Rig r;
  r.warmUp();
  r.send(PadCmdOp::ARM);
  r.send(PadCmdOp::LAUNCH);
  r.run(5.0);
  r.groundAlive = false;
  r.run(1.5);
  CHECK(r.pad.state() == PadState::SAFE);
  CHECK(r.pad.lastReason() == PadReason::GROUND_LINK_LOST);
  CHECK(!r.pad.ignitionUsed());
}

void reachIgnition(Rig& r) {
  r.warmUp();
  r.send(PadCmdOp::ARM);
  r.send(PadCmdOp::LAUNCH);
  r.run(30.1);
}

void testHangFire() {
  Rig r;
  reachIgnition(r);
  CHECK(r.pad.state() == PadState::IGNITION);
  r.run(2.1);  // 點火窗內沒有推力
  CHECK(r.pad.state() == PadState::HANG_FIRE);
  CHECK(r.pad.lastReason() == PadReason::NO_THRUST);
  CHECK(!r.out.igniterOn && !r.out.clampRelease);  // 點火關、夾持保持鎖死
  r.run(5.0);
  CHECK(r.pad.state() == PadState::HANG_FIRE);  // 不自動重試
  CHECK(r.pad.ignitionUsed());
  CHECK(r.send(PadCmdOp::ARM) == PadCmdResult::REJECTED);
  CHECK(r.ev.lastCmdReason == PadReason::IGNITION_LOCKED);  // 一次開機只點一次
}

void testGroundLossDuringIgnitionContinues() {
  Rig r;
  reachIgnition(r);
  r.groundAlive = false;
  r.run(1.5);  // 地面斷線超過 groundLinkTimeout_s
  CHECK(r.pad.state() == PadState::IGNITION);
  r.in.thrustOk = true;
  r.run(0.2);
  CHECK(r.pad.state() == PadState::RELEASED);
}

void testAbortDuringIgnitionHoldsClamp() {
  Rig r;
  reachIgnition(r);
  r.in.thrustOk = true;  // 推力剛起來，還沒滿確認時間
  r.run(0.03);
  CHECK(r.send(PadCmdOp::ABORT) == PadCmdResult::ACCEPTED);
  CHECK(r.pad.state() == PadState::HOLD_ABORT);
  r.run(1.0);
  CHECK(!r.out.igniterOn && !r.out.clampRelease);
}

void testKeyPulledDuringIgnition() {
  Rig r;
  reachIgnition(r);
  r.in.keyOn = false;
  r.step();
  CHECK(r.pad.state() == PadState::HOLD_ABORT);
  CHECK(r.pad.lastReason() == PadReason::KEY_OFF);
  CHECK(!r.out.igniterOn && !r.out.clampRelease);
  CHECK(!r.everIgniterWithoutKey);
}

void testNoLiftoffAfterRelease() {
  Rig r;
  reachIgnition(r);
  r.in.thrustOk = true;
  r.run(0.2);
  CHECK(r.pad.state() == PadState::RELEASED);
  r.in.thrustOk = false;
  r.run(3.1);  // 鬆了但沒離軌、臍帶也還在
  CHECK(r.pad.state() == PadState::NO_LIFTOFF);
  CHECK(r.out.clampRelease);  // 鬆了就不能再夾回
  CHECK(!r.clampReleasedThenLocked);
  CHECK(r.send(PadCmdOp::ARM) == PadCmdResult::REJECTED);
  // ABORT 在 RELEASED 時拒絕（不能夾回）已在 handleCommand 註明；終態下 ABORT 視為已安全
  CHECK(r.send(PadCmdOp::ABORT) == PadCmdResult::ACCEPTED);
}

void testUmbilicalTxReadyRules() {
  struct NullUart : IUmbilicalUart {
    bool ok = true;
    int write(const std::uint8_t*, std::size_t n) override { return ok ? static_cast<int>(n) : -1; }
  } u;
  UmbilicalTxCore tx(u);
  CHECK(!tx.computeReady(0.0));  // 還沒有 BIT
  tx.setBit(kRocketBitHardMask, 0.0);
  CHECK(tx.computeReady(0.1));
  CHECK(!tx.computeReady(1.0));  // BIT 太舊
  tx.setBit(kRocketBitHardMask & static_cast<std::uint16_t>(~BIT_SERVO_NEUTRAL_OK), 1.0);
  CHECK(!tx.computeReady(1.0));  // 硬擋沒過
  tx.setBit(kRocketBitHardMask, 1.0);  // SD／遙測警告不擋
  CHECK(tx.computeReady(1.0));
  tx.setMode(2);  // LIFTOFF：已經不在架上
  CHECK(!tx.computeReady(1.0));
  // 起飛後臍帶斷：寫入失敗只計數
  u.ok = false;
  CHECK(tx.tick(2.0));
  CHECK(!tx.tick(2.05));  // 10 Hz 節流
  CHECK(tx.tick(2.1));
  CHECK(tx.writeFailures() == 2u);
}

}  // namespace

int main() {
  testCodecRoundTripAndCrc();
  testEveryMessageFitsNrf24();
  testStreamDeframerResync();
  testCmdGate();
  testNominalLaunch();
  testArmRejectedWhenAnyRed();
  testNoGroundLinkNoArm();
  testLaunchNeedsArmFirst();
  testRedDuringCountdownNeedsFullRearm();
  testGroundLossDuringCountdownAborts();
  testHangFire();
  testGroundLossDuringIgnitionContinues();
  testAbortDuringIgnitionHoldsClamp();
  testKeyPulledDuringIgnition();
  testNoLiftoffAfterRelease();
  testUmbilicalTxReadyRules();
  if (g_fail == 0) {
    std::printf("PadLinkHostTest: all 16 tests passed\n");
    return 0;
  }
  std::printf("PadLinkHostTest: %d CHECK(s) FAILED\n", g_fail);
  return 1;
}
