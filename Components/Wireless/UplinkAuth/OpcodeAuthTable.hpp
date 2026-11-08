#ifndef UPLINKAUTH_OPCODEAUTHTABLE_HPP
#define UPLINKAUTH_OPCODEAUTHTABLE_HPP

#include <cstddef>
#include <cstdint>

namespace UplinkAuth {

//! HMAC policy: **deny-by-default** — every opcode requires HMAC unless listed
//! in this exemption table. Unknown / newly invented opcodes always require HMAC.
//!
//! Provisional constants below are **待 Edward 定** (not flight-frozen values).
struct OpcodeAuthTable {
    static constexpr std::size_t kMaxEntries = 8;

    //! Provisional abort / override-class opcode — **待 Edward 定**.
    //! Named constant only; NOT an exemption. Default policy still requires HMAC.
    static constexpr std::uint16_t kProvisionalAbortOpcode = 0x0001;

    //! Provisional US gimbal (雲台) servo-control opcode — **待 Edward 定**.
    //! Sole default HMAC exemption. Exemption ≠ whitelist bypass (§8 still applies).
    static constexpr std::uint16_t kProvisionalGimbalOpcode = 0x0200;

    std::uint16_t exemptOpcodes[kMaxEntries];
    std::size_t count;

    //! Default: only gimbal is exempt. **待 Edward 定**.
    static OpcodeAuthTable makeDefault() {
        OpcodeAuthTable t;
        t.count = 1;
        t.exemptOpcodes[0] = kProvisionalGimbalOpcode;
        return t;
    }

    bool isExempt(std::uint16_t opcode) const {
        for (std::size_t i = 0; i < count && i < kMaxEntries; ++i) {
            if (exemptOpcodes[i] == opcode) {
                return true;
            }
        }
        return false;
    }

    //! Inverse of exemption: unknown opcodes always return true.
    bool requiresHmac(std::uint16_t opcode) const { return !isExempt(opcode); }

    void setExemptOpcodes(const std::uint16_t* list, std::size_t n) {
        count = (n > kMaxEntries) ? kMaxEntries : n;
        for (std::size_t i = 0; i < count; ++i) {
            exemptOpcodes[i] = list[i];
        }
    }

    bool addExempt(std::uint16_t opcode) {
        if (isExempt(opcode)) {
            return true;
        }
        if (count >= kMaxEntries) {
            return false;
        }
        exemptOpcodes[count++] = opcode;
        return true;
    }
};

}  // namespace UplinkAuth

#endif
