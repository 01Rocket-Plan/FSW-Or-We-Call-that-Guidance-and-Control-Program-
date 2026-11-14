#include "WhitelistSimTable.hpp"

namespace UplinkAuth {
namespace Whitelist {

namespace {
constexpr uint8_t modeBit(ModeClass m) {
  return static_cast<uint8_t>(1u << static_cast<uint8_t>(m));
}
constexpr uint8_t vehicleBit(VehicleRole v) {
  return static_cast<uint8_t>(1u << static_cast<uint8_t>(v));
}
}  // namespace

WhitelistSimTable::WhitelistSimTable() {
  for (size_t i = 0; i < kMaxEntries; ++i) {
    m_entries[i] = WhitelistEntry{};
  }

  // OPCODE_PARAM: allowed on PAD only (both vehicles); needs HMAC.
  {
    WhitelistEntry& e = m_entries[0];
    e.used = true;
    e.opcode = kOpcodeParam;
    e.modeMask = modeBit(ModeClass::PAD);
    e.vehicleMask = static_cast<uint8_t>(vehicleBit(VehicleRole::DS) | vehicleBit(VehicleRole::US));
    e.hmacExempt = false;
    e.expectedArgLen = 2;
    e.hasParamRange = true;
    e.paramMin = -100;
    e.paramMax = 100;
  }

  // OPCODE_TEST: PAD only; needs HMAC.
  {
    WhitelistEntry& e = m_entries[1];
    e.used = true;
    e.opcode = kOpcodeTest;
    e.modeMask = modeBit(ModeClass::PAD);
    e.vehicleMask = static_cast<uint8_t>(vehicleBit(VehicleRole::DS) | vehicleBit(VehicleRole::US));
    e.hmacExempt = false;
    e.expectedArgLen = 0;
    e.hasParamRange = false;
  }

  // OPCODE_AUTH_DEMO: any listed mode; needs HMAC (AUTH_REQUIRED until verified).
  {
    WhitelistEntry& e = m_entries[2];
    e.used = true;
    e.opcode = kOpcodeAuthDemo;
    e.modeMask = static_cast<uint8_t>(modeBit(ModeClass::PAD) |
                                     modeBit(ModeClass::AFTER_LIFTOFF));
    e.vehicleMask = static_cast<uint8_t>(vehicleBit(VehicleRole::DS) | vehicleBit(VehicleRole::US));
    e.hmacExempt = false;
    e.expectedArgLen = UINT16_MAX;
    e.hasParamRange = false;
  }

  // Gimbal: US only, HMAC-exempt, angle int16 in [-90, 90] deg placeholder range.
  {
    WhitelistEntry& e = m_entries[3];
    e.used = true;
    e.opcode = kOpcodeGimbal;
    e.modeMask = static_cast<uint8_t>(modeBit(ModeClass::PAD) |
                                     modeBit(ModeClass::AFTER_LIFTOFF));
    e.vehicleMask = vehicleBit(VehicleRole::US);
    e.hmacExempt = true;
    e.expectedArgLen = 2;
    e.hasParamRange = true;
    e.paramMin = -90;
    e.paramMax = 90;
  }
}

const WhitelistEntry* WhitelistSimTable::find(uint16_t opcode) const {
  for (size_t i = 0; i < kMaxEntries; ++i) {
    if (m_entries[i].used && m_entries[i].opcode == opcode) {
      return &m_entries[i];
    }
  }
  return nullptr;
}

}  // namespace Whitelist
}  // namespace UplinkAuth
