// WhitelistGate: uplink command table gate (host-testable, no F´ dependency).
// Policy/table content owned by MCU/FlightMode — this module only enforces lookup.
// Does not implement HMAC, read keys, or mutate flight-mode state.
#ifndef WIRELESS_UPLINKAUTH_WHITELISTGATE_HPP
#define WIRELESS_UPLINKAUTH_WHITELISTGATE_HPP

#include <cstddef>
#include <cstdint>

namespace UplinkAuth {
namespace Whitelist {

// Placeholder mode ids for host tests / table config (not flight thresholds).
enum class FlightModeId : uint8_t {
  MODE_PAD = 0,
  MODE_BOOST_AFTER = 1,
  MODE_COUNT = 2
};

// DS vs US deployment role (雲台 only accepted on US per link contract).
enum class VehicleRole : uint8_t {
  DS = 0,
  US = 1
};

enum class GateDecision : uint8_t {
  ALLOW = 0,
  DENY = 1,
  AUTH_REQUIRED = 2
};

// Deny reason for tests / events (EVT_CMD_REJECTED on DENY paths).
enum class DenyReason : uint8_t {
  None = 0,
  UnknownOpcode = 1,
  ModeDenied = 2,
  VehicleDenied = 3,
  MalformedLength = 4,
  ParamOutOfRange = 5,
  AuthMissing = 6
};

struct CmdView {
  uint16_t opcode = 0;
  FlightModeId mode = FlightModeId::MODE_PAD;
  VehicleRole vehicle = VehicleRole::DS;
  const uint8_t* args = nullptr;
  uint16_t argLen = 0;
  // Set by the in-group HMAC block: true if tag verified, or opcode is
  // HMAC-exempt and earlier pipeline stages already passed.
  bool authVerified = false;
};

class ICmdSink {
 public:
  virtual ~ICmdSink() = default;
  virtual void onCommand(const CmdView& cmd) = 0;
};

class IGateEventSink {
 public:
  virtual ~IGateEventSink() = default;
  virtual void onCmdRejected(const CmdView& cmd, DenyReason reason) = 0;
};

// One configurable row. Unknown opcodes (no row) are DENY.
struct WhitelistEntry {
  uint16_t opcode = 0;
  bool used = false;
  // Bitmask of allowed FlightModeId values (bit 0 = MODE_PAD, ...).
  uint8_t modeMask = 0;
  // Bitmask of allowed VehicleRole (bit 0 = DS, bit 1 = US).
  uint8_t vehicleMask = 0;
  // If true, gate may ALLOW without authVerified (HMAC-exempt opcodes only).
  // Edward rule: only US gimbal servo is HMAC-exempt; still must pass this row.
  bool hmacExempt = false;
  // Expected argument length. UINT16_MAX = any length accepted.
  uint16_t expectedArgLen = 0;
  // Optional inclusive range over a big-endian int16 at args[0..1].
  bool hasParamRange = false;
  int16_t paramMin = 0;
  int16_t paramMax = 0;
};

class IWhitelistTable {
 public:
  virtual ~IWhitelistTable() = default;
  // Returns nullptr if opcode is unknown → caller must DENY.
  virtual const WhitelistEntry* find(uint16_t opcode) const = 0;
};

class WhitelistGate {
 public:
  WhitelistGate(const IWhitelistTable& table, ICmdSink& cmd, IGateEventSink& events);

  void tick();

  // Deny-by-default: unknown opcode → DENY.
  // HMAC-exempt row + checks pass → ALLOW (authVerified ignored).
  // Non-exempt row + checks pass + authVerified → ALLOW.
  // Non-exempt row + checks pass + !authVerified → AUTH_REQUIRED (no forward).
  GateDecision handle(const CmdView& cmd);

  uint32_t allowedCount() const { return m_allowed; }
  uint32_t deniedCount() const { return m_denied; }
  uint32_t authRequiredCount() const { return m_authRequired; }
  DenyReason lastDenyReason() const { return m_lastDeny; }

 private:
  GateDecision deny(const CmdView& cmd, DenyReason reason);
  GateDecision allow(const CmdView& cmd);
  static bool modeAllowed(const WhitelistEntry& e, FlightModeId mode);
  static bool vehicleAllowed(const WhitelistEntry& e, VehicleRole v);
  static bool lengthOk(const WhitelistEntry& e, uint16_t argLen);
  static bool paramOk(const WhitelistEntry& e, const uint8_t* args, uint16_t argLen);

  const IWhitelistTable& m_table;
  ICmdSink& m_cmd;
  IGateEventSink& m_events;
  uint32_t m_allowed = 0;
  uint32_t m_denied = 0;
  uint32_t m_authRequired = 0;
  DenyReason m_lastDeny = DenyReason::None;
};

}  // namespace Whitelist
}  // namespace UplinkAuth

#endif
