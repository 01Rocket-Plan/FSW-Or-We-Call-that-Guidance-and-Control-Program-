#ifndef UPLINKAUTH_OPCODEAUTHTABLE_HPP
#define UPLINKAUTH_OPCODEAUTHTABLE_HPP

#include <cstddef>
#include <cstdint>

namespace UplinkAuth {

//! Configurable table of opcodes that require HMAC verification.
//! Values below are **待 Edward 定** (provisional placeholders only).
struct OpcodeAuthTable {
    static constexpr std::size_t kMaxEntries = 8;

    std::uint16_t opcodes[kMaxEntries];
    std::size_t count;

    //! Provisional placeholder opcode requiring HMAC — **待 Edward 定**.
    //! Lives only in this parameter table; call sites use the table, not bare literals.
    static constexpr std::uint16_t kProvisionalHmacOpcode0 = 0x0001;

    //! Default table: single provisional entry. **待 Edward 定** — not a flight constant.
    static OpcodeAuthTable makeDefault() {
        OpcodeAuthTable t;
        t.count = 1;
        t.opcodes[0] = kProvisionalHmacOpcode0;
        return t;
    }

    bool requiresHmac(std::uint16_t opcode) const {
        for (std::size_t i = 0; i < count && i < kMaxEntries; ++i) {
            if (opcodes[i] == opcode) {
                return true;
            }
        }
        return false;
    }

    void setOpcodes(const std::uint16_t* list, std::size_t n) {
        count = (n > kMaxEntries) ? kMaxEntries : n;
        for (std::size_t i = 0; i < count; ++i) {
            opcodes[i] = list[i];
        }
    }

    bool addOpcode(std::uint16_t opcode) {
        if (requiresHmac(opcode)) {
            return true;
        }
        if (count >= kMaxEntries) {
            return false;
        }
        opcodes[count++] = opcode;
        return true;
    }
};

}  // namespace UplinkAuth

#endif
