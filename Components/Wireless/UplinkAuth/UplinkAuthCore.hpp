#ifndef UPLINKAUTH_UPLINKAUTHCORE_HPP
#define UPLINKAUTH_UPLINKAUTHCORE_HPP

#include "IAuthNvStore.hpp"
#include "OpcodeAuthTable.hpp"

#include <cstddef>
#include <cstdint>

namespace UplinkAuth {

enum class AuthStatus {
    Ok = 0,
    Replay,
    AuthFail,
    MissingKey,
    BadInput,
    StoreFail,
    CounterInvalid
};

typedef void (*ForwardFn)(void* ctx);

//! Anti-replay + HMAC-SHA256 for table-listed opcodes.
//! HMAC-required opcode set / provisional values: **待 Edward 定**.
class UplinkAuthCore {
  public:
    static constexpr std::size_t kTagBytes = 32;
    static constexpr std::size_t kMaxPskBytes = 64;
    static constexpr std::size_t kMaxArgsBytes = 256;
    static constexpr const char* kDomainTag = "01RP-UPL-v1";
    static constexpr std::size_t kDomainTagLen = 11;
    static constexpr std::uint32_t kCmdCounterForbidden = 0xFFFFFFFFu;
    static constexpr std::uint16_t kProvisionalHmacOpcode0 =
        OpcodeAuthTable::kProvisionalHmacOpcode0;
    static constexpr std::size_t kMacHeaderBytes = 11 + 1 + 4 + 2 + 2;

    explicit UplinkAuthCore(IAuthNvStore& store);
    AuthStatus init();

    const OpcodeAuthTable& hmacTable() const { return m_hmacTable; }
    OpcodeAuthTable& hmacTable() { return m_hmacTable; }
    void setHmacTable(const OpcodeAuthTable& table) { m_hmacTable = table; }

    AuthStatus accept(std::uint32_t cmdCounter, std::uint16_t opcode,
                      std::uint8_t vehicleId, const std::uint8_t* args,
                      std::uint16_t argLen, const std::uint8_t* authTag,
                      std::size_t authTagLen, ForwardFn onForward, void* forwardCtx);

    std::uint32_t lastCmdCounter() const { return m_lastCmdCounter; }
    bool pskLoaded() const { return m_pskLen > 0; }

    static std::size_t buildMacMessage(const char* domainTag, std::size_t domainTagLen,
                                       std::uint8_t vehicleId, std::uint32_t cmdCounter,
                                       std::uint16_t opcode, const std::uint8_t* args,
                                       std::uint16_t argLen, std::uint8_t* out,
                                       std::size_t outMax);

  private:
    IAuthNvStore& m_store;
    OpcodeAuthTable m_hmacTable;
    std::uint32_t m_lastCmdCounter = 0;
    std::uint8_t m_psk[kMaxPskBytes];
    std::size_t m_pskLen = 0;
};

}  // namespace UplinkAuth

#endif
