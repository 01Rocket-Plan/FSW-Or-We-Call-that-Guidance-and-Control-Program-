// Host-only fake table. No real flight thresholds or mission constants.
#ifndef WIRELESS_UPLINKAUTH_WHITELISTSIMTABLE_HPP
#define WIRELESS_UPLINKAUTH_WHITELISTSIMTABLE_HPP

#include "WhitelistGate.hpp"

namespace UplinkAuth {
namespace Whitelist {

class WhitelistSimTable : public IWhitelistTable {
 public:
  static constexpr size_t kMaxEntries = 16;

  // Provisional placeholders — 待 Edward 定 (align with HMAC exemption constant).
  static constexpr uint16_t kOpcodeParam = 0x0010;
  static constexpr uint16_t kOpcodeTest = 0x0011;
  static constexpr uint16_t kOpcodeAuthDemo = 0x0012;
  static constexpr uint16_t kOpcodeGimbal = 0x0200;  // US gimbal servo; HMAC-exempt

  WhitelistSimTable();
  const WhitelistEntry* find(uint16_t opcode) const override;

 private:
  WhitelistEntry m_entries[kMaxEntries];
};

}  // namespace Whitelist
}  // namespace UplinkAuth

#endif
