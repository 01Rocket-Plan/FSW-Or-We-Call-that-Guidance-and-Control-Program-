// Host tests for WhitelistGate (fake table only; no F´ / no keys).
#include "Components/Wireless/UplinkAuth/whitelist/WhitelistGate.hpp"
#include "Components/Wireless/UplinkAuth/whitelist/WhitelistSimTable.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace UplinkAuth::Whitelist;

namespace {

#define CHECK(cond)                                                                         \
  do {                                                                                      \
    if (!(cond)) {                                                                          \
      std::fprintf(stderr, "CHECK failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__);        \
      std::abort();                                                                         \
    }                                                                                       \
  } while (0)

struct CmdRecorder : public ICmdSink {
  uint32_t n = 0;
  CmdView last{};
  void onCommand(const CmdView& cmd) override {
    n++;
    last = cmd;
  }
};

struct RejectRecorder : public IGateEventSink {
  uint32_t n = 0;
  DenyReason lastReason = DenyReason::None;
  uint16_t lastOpcode = 0;
  void onCmdRejected(const CmdView& cmd, DenyReason reason) override {
    n++;
    lastReason = reason;
    lastOpcode = cmd.opcode;
  }
};

struct SimFlightState {
  int modeVar = 7;
  int armVar = 0;
  uint32_t seq = 0;
};

void beInt16(int16_t v, uint8_t out[2]) {
  out[0] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
  out[1] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);
}

CmdView makeCmd(uint16_t opcode, FlightModeId mode, VehicleRole vehicle, bool auth,
                const uint8_t* args = nullptr, uint16_t argLen = 0) {
  CmdView c;
  c.opcode = opcode;
  c.mode = mode;
  c.vehicle = vehicle;
  c.args = args;
  c.argLen = argLen;
  c.authVerified = auth;
  return c;
}

void testUnknownOpcodeDenied(WhitelistGate& g, CmdRecorder& cmd, RejectRecorder& rej) {
  const uint32_t c0 = cmd.n;
  const uint32_t r0 = rej.n;
  const GateDecision d = g.handle(makeCmd(0xDEAD, FlightModeId::PAD_IDLE, VehicleRole::US, true));
  CHECK(d == GateDecision::DENY);
  CHECK(cmd.n == c0);
  CHECK(rej.n == r0 + 1);
  CHECK(rej.lastReason == DenyReason::UnknownOpcode);
  CHECK(rej.lastOpcode == 0xDEAD);
}

void testBoostParamDenied(WhitelistGate& g, CmdRecorder& cmd, RejectRecorder& rej,
                          SimFlightState& sim) {
  const SimFlightState snap = sim;
  uint8_t args[2];
  beInt16(0, args);
  const uint32_t c0 = cmd.n;
  const GateDecision d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeParam, FlightModeId::COAST, VehicleRole::DS,
              true, args, 2));
  CHECK(d == GateDecision::DENY);
  CHECK(cmd.n == c0);
  CHECK(rej.lastReason == DenyReason::ModeDenied);
  CHECK(sim.modeVar == snap.modeVar);
  CHECK(sim.armVar == snap.armVar);
  CHECK(sim.seq == snap.seq);
}

void testPadTestNeedsAuthThenAllow(WhitelistGate& g, CmdRecorder& cmd) {
  const uint32_t c0 = cmd.n;
  GateDecision d =
      g.handle(makeCmd(WhitelistSimTable::kOpcodeTest, FlightModeId::PAD_IDLE, VehicleRole::DS,
                       false));
  CHECK(d == GateDecision::AUTH_REQUIRED);
  CHECK(cmd.n == c0);

  d = g.handle(makeCmd(WhitelistSimTable::kOpcodeTest, FlightModeId::PAD_IDLE, VehicleRole::DS,
                       true));
  CHECK(d == GateDecision::ALLOW);
  CHECK(cmd.n == c0 + 1);
  CHECK(cmd.last.opcode == WhitelistSimTable::kOpcodeTest);
}

void testAuthDemoNotDirectlyForwarded(WhitelistGate& g, CmdRecorder& cmd) {
  const uint32_t c0 = cmd.n;
  const GateDecision d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeAuthDemo, FlightModeId::PAD_IDLE, VehicleRole::US, false));
  CHECK(d == GateDecision::AUTH_REQUIRED);
  CHECK(cmd.n == c0);
}

void testMalformedLength(WhitelistGate& g, CmdRecorder& cmd, RejectRecorder& rej) {
  const uint32_t c0 = cmd.n;
  uint8_t one = 0x00;
  GateDecision d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeParam, FlightModeId::PAD_IDLE, VehicleRole::DS, true, &one,
              1));
  CHECK(d == GateDecision::DENY);
  CHECK(cmd.n == c0);
  CHECK(rej.lastReason == DenyReason::MalformedLength);

  d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeParam, FlightModeId::PAD_IDLE, VehicleRole::DS, true,
              nullptr, 2));
  CHECK(d == GateDecision::DENY);
  CHECK(rej.lastReason == DenyReason::MalformedLength);
}

void testParamOutOfRange(WhitelistGate& g, CmdRecorder& cmd, RejectRecorder& rej) {
  uint8_t args[2];
  beInt16(101, args);
  const uint32_t c0 = cmd.n;
  const GateDecision d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeParam, FlightModeId::PAD_IDLE, VehicleRole::DS, true, args,
              2));
  CHECK(d == GateDecision::DENY);
  CHECK(cmd.n == c0);
  CHECK(rej.lastReason == DenyReason::ParamOutOfRange);
}

void testGimbalInRangeAcceptedWithoutAuth(WhitelistGate& g, CmdRecorder& cmd) {
  uint8_t args[2];
  beInt16(45, args);
  const uint32_t c0 = cmd.n;
  const GateDecision d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeGimbal, FlightModeId::COAST, VehicleRole::US,
              false, args, 2));
  CHECK(d == GateDecision::ALLOW);
  CHECK(cmd.n == c0 + 1);
  CHECK(cmd.last.opcode == WhitelistSimTable::kOpcodeGimbal);
}

void testGimbalOutOfRangeDenied(WhitelistGate& g, CmdRecorder& cmd, RejectRecorder& rej) {
  uint8_t args[2];
  beInt16(91, args);
  const uint32_t c0 = cmd.n;
  const GateDecision d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeGimbal, FlightModeId::PAD_IDLE, VehicleRole::US, false,
              args, 2));
  CHECK(d == GateDecision::DENY);
  CHECK(cmd.n == c0);
  CHECK(rej.lastReason == DenyReason::ParamOutOfRange);
}

void testGimbalOnDsDenied(WhitelistGate& g, CmdRecorder& cmd, RejectRecorder& rej) {
  uint8_t args[2];
  beInt16(0, args);
  const uint32_t c0 = cmd.n;
  const GateDecision d = g.handle(
      makeCmd(WhitelistSimTable::kOpcodeGimbal, FlightModeId::PAD_IDLE, VehicleRole::DS, false,
              args, 2));
  CHECK(d == GateDecision::DENY);
  CHECK(cmd.n == c0);
  CHECK(rej.lastReason == DenyReason::VehicleDenied);
}

}  // namespace

static void testModeClassMapping() {
  // 18 個 RocketTypes.FlightModeId → 粗分類（#37-2），逐一列出，新增值時這裡會提醒要歸類。
  struct Row {
    FlightModeId::T mode;
    ModeClass cls;
  };
  const Row rows[] = {
      {FlightModeId::PAD_IDLE, ModeClass::PAD},
      {FlightModeId::ARMED, ModeClass::PAD},
      {FlightModeId::LIFTOFF, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::BOOST_PASSIVE, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::BOOST_ACTIVE, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::COAST, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::SEPARATED, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::LS_DESCENT, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::LS_CHUTE, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::LS_LANDED, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::HANGFIRE, ModeClass::PAD},
      {FlightModeId::SEP_FAIL, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::ABORT, ModeClass::AFTER_LIFTOFF},
      // US_WAIT＝上級分離前等待（墊上＋DS 推進／滑翔都在這個模式）→ fail-safe 當 AFTER_LIFTOFF
      {FlightModeId::US_WAIT, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::US_COAST, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::US_CHUTE, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::US_LANDED, ModeClass::AFTER_LIFTOFF},
      {FlightModeId::US_CHUTE_INHIBIT, ModeClass::AFTER_LIFTOFF},
  };
  CHECK(sizeof(rows) / sizeof(rows[0]) == FlightModeId::NUM_CONSTANTS);
  for (const Row& r : rows) {
    CHECK(modeClassOf(FlightModeId(r.mode)) == r.cls);
  }
}

// US_WAIT 時（上級可能已在空中）墊上專用指令必須被擋。
static void testUsWaitBlocksPadOnlyCommands() {
  WhitelistSimTable table;
  CmdRecorder cmd;
  RejectRecorder rej;
  WhitelistGate g(table, cmd, rej);
  uint8_t args[2];
  beInt16(0, args);
  const GateDecision d = g.handle(makeCmd(WhitelistSimTable::kOpcodeParam, FlightModeId::US_WAIT,
                                          VehicleRole::US, true, args, 2));
  CHECK(d == GateDecision::DENY);
  CHECK(cmd.n == 0u);
}

static int WhitelistLegacyMain() {
  testModeClassMapping();
  testUsWaitBlocksPadOnlyCommands();
  WhitelistSimTable table;
  CmdRecorder cmd;
  RejectRecorder rej;
  SimFlightState sim;
  WhitelistGate gate(table, cmd, rej);

  testUnknownOpcodeDenied(gate, cmd, rej);
  testBoostParamDenied(gate, cmd, rej, sim);
  testPadTestNeedsAuthThenAllow(gate, cmd);
  testAuthDemoNotDirectlyForwarded(gate, cmd);
  testMalformedLength(gate, cmd, rej);
  testParamOutOfRange(gate, cmd, rej);
  testGimbalInRangeAcceptedWithoutAuth(gate, cmd);
  testGimbalOutOfRangeDenied(gate, cmd, rej);
  testGimbalOnDsDenied(gate, cmd, rej);

  CHECK(gate.allowedCount() == 2);
  CHECK(gate.deniedCount() >= 5);
  CHECK(gate.authRequiredCount() == 2);

  std::printf("WhitelistGate host tests: PASS\n");
  return 0;
}

// 真 F´ UT（gtest，ctest 會跑）；-DHOST_TEST_MAIN 時仍可用舊的 run_host_tests.sh 快速自測
#ifdef HOST_TEST_MAIN
int main() { return WhitelistLegacyMain(); }
#else
#include <gtest/gtest.h>
TEST(Wireless, WhitelistHostSuite) { ASSERT_EQ(0, WhitelistLegacyMain()); }
#endif
