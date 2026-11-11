#include "WhitelistGate.hpp"

namespace UplinkAuth {
namespace Whitelist {

WhitelistGate::WhitelistGate(const IWhitelistTable& table, ICmdSink& cmd, IGateEventSink& events)
    : m_table(table), m_cmd(cmd), m_events(events) {}

void WhitelistGate::tick() {
  // No internal queue today; kept for non-blocking cadence parity with F´ components.
}

ModeClass modeClassOf(FlightModeId mode) {
  switch (mode.e) {
    case FlightModeId::PAD_IDLE:
    case FlightModeId::ARMED:
    case FlightModeId::HANGFIRE:
      return ModeClass::PAD;
    // US_WAIT（上級「分離前等待」）橫跨墊上＋DS 推進／滑翔，上級自己分不出是否已離台，
    // 所以一律當 AFTER_LIFTOFF（fail-safe）：墊上專用指令（參數／測試）在飛行中不能進上級。
    // 若上級墊上真的需要這類指令，要由 DS／臍帶提供「仍在架上」旗標再細分（待 MCU 組／Edward）。
    default:
      return ModeClass::AFTER_LIFTOFF;
  }
}

bool WhitelistGate::modeAllowed(const WhitelistEntry& e, FlightModeId mode) {
  const uint8_t bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(modeClassOf(mode)));
  return (e.modeMask & bit) != 0;
}

bool WhitelistGate::vehicleAllowed(const WhitelistEntry& e, VehicleRole v) {
  const uint8_t bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(v));
  return (e.vehicleMask & bit) != 0;
}

bool WhitelistGate::lengthOk(const WhitelistEntry& e, uint16_t argLen) {
  if (e.expectedArgLen == UINT16_MAX) {
    return true;
  }
  return argLen == e.expectedArgLen;
}

bool WhitelistGate::paramOk(const WhitelistEntry& e, const uint8_t* args, uint16_t argLen) {
  if (!e.hasParamRange) {
    return true;
  }
  if (args == nullptr || argLen < 2) {
    return false;
  }
  const int16_t value = static_cast<int16_t>((static_cast<uint16_t>(args[0]) << 8) |
                                             static_cast<uint16_t>(args[1]));
  return value >= e.paramMin && value <= e.paramMax;
}

GateDecision WhitelistGate::deny(const CmdView& cmd, DenyReason reason) {
  m_denied++;
  m_lastDeny = reason;
  m_events.onCmdRejected(cmd, reason);
  return GateDecision::DENY;
}

GateDecision WhitelistGate::allow(const CmdView& cmd) {
  m_allowed++;
  m_lastDeny = DenyReason::None;
  m_cmd.onCommand(cmd);
  return GateDecision::ALLOW;
}

GateDecision WhitelistGate::handle(const CmdView& cmd) {
  // Bounds: reject absurd argLen even before table lookup when pointer is null.
  if (cmd.argLen > 0 && cmd.args == nullptr) {
    return deny(cmd, DenyReason::MalformedLength);
  }

  const WhitelistEntry* entry = m_table.find(cmd.opcode);
  if (entry == nullptr || !entry->used) {
    return deny(cmd, DenyReason::UnknownOpcode);
  }
  if (!modeAllowed(*entry, cmd.mode)) {
    return deny(cmd, DenyReason::ModeDenied);
  }
  if (!vehicleAllowed(*entry, cmd.vehicle)) {
    return deny(cmd, DenyReason::VehicleDenied);
  }
  if (!lengthOk(*entry, cmd.argLen)) {
    return deny(cmd, DenyReason::MalformedLength);
  }
  if (!paramOk(*entry, cmd.args, cmd.argLen)) {
    return deny(cmd, DenyReason::ParamOutOfRange);
  }

  // Edward rule (HMAC layer): default require auth; only hmacExempt rows skip it.
  // Exempt rows still must pass the checks above (whitelist + ranges).
  if (entry->hmacExempt) {
    return allow(cmd);
  }
  if (!cmd.authVerified) {
    m_authRequired++;
    m_lastDeny = DenyReason::AuthMissing;
    return GateDecision::AUTH_REQUIRED;
  }
  return allow(cmd);
}

}  // namespace Whitelist
}  // namespace UplinkAuth
