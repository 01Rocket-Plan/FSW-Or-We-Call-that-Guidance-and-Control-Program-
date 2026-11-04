#include "UplinkAuthCore.hpp"
#include "HmacSha256.hpp"

#include <cstring>

namespace UplinkAuth {

namespace {

void storeBe32(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>((v >> 24) & 0xFFu);
    p[1] = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
    p[2] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
    p[3] = static_cast<std::uint8_t>(v & 0xFFu);
}

void storeBe16(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
    p[1] = static_cast<std::uint8_t>(v & 0xFFu);
}

}  // namespace

UplinkAuthCore::UplinkAuthCore(IAuthNvStore& store)
    : m_store(store), m_hmacExempt(OpcodeAuthTable::makeDefault()) {
    std::memset(m_psk, 0, sizeof(m_psk));
}

AuthStatus UplinkAuthCore::init() {
    std::uint32_t last = 0;
    if (!m_store.readLastCmdCounter(last)) {
        last = 0;
    }
    m_lastCmdCounter = last;
    // PSK only from external NVS / config interface — never a compiled-in flight key.
    std::size_t pskLen = 0;
    if (m_store.readPsk(m_psk, kMaxPskBytes, pskLen) && pskLen > 0) {
        m_pskLen = pskLen;
    } else {
        m_pskLen = 0;
        std::memset(m_psk, 0, sizeof(m_psk));
    }
    return AuthStatus::Ok;
}

std::size_t UplinkAuthCore::buildMacMessage(const char* domainTag, std::size_t domainTagLen,
                                            std::uint8_t vehicleId, std::uint32_t cmdCounter,
                                            std::uint16_t opcode, const std::uint8_t* args,
                                            std::uint16_t argLen, std::uint8_t* out,
                                            std::size_t outMax) {
    if (domainTag == nullptr || domainTagLen == 0) {
        return 0;
    }
    if (argLen > kMaxArgsBytes) {
        return 0;
    }
    if (argLen > 0 && args == nullptr) {
        return 0;
    }
    const std::size_t need = domainTagLen + 1u + 4u + 2u + 2u + static_cast<std::size_t>(argLen);
    if (out == nullptr || outMax < need) {
        return 0;
    }
    std::size_t off = 0;
    std::memcpy(out + off, domainTag, domainTagLen);
    off += domainTagLen;
    out[off++] = vehicleId;
    storeBe32(out + off, cmdCounter);
    off += 4;
    storeBe16(out + off, opcode);
    off += 2;
    storeBe16(out + off, argLen);
    off += 2;
    if (argLen > 0) {
        std::memcpy(out + off, args, argLen);
        off += argLen;
    }
    return off;
}

AuthStatus UplinkAuthCore::accept(std::uint32_t cmdCounter, std::uint16_t opcode,
                                  std::uint8_t vehicleId, const std::uint8_t* args,
                                  std::uint16_t argLen, const std::uint8_t* authTag,
                                  std::size_t authTagLen, ForwardFn onForward,
                                  void* forwardCtx) {
    if (cmdCounter == kCmdCounterForbidden) {
        return AuthStatus::CounterInvalid;
    }
    if (argLen > kMaxArgsBytes) {
        return AuthStatus::BadInput;
    }
    if (argLen > 0 && args == nullptr) {
        return AuthStatus::BadInput;
    }
    if (cmdCounter <= m_lastCmdCounter) {
        return AuthStatus::Replay;
    }

    // Deny-by-default: require HMAC unless exempt (gimbal only in default table).
    if (m_hmacExempt.requiresHmac(opcode)) {
        if (m_pskLen == 0) {
            return AuthStatus::MissingKey;
        }
        if (authTag == nullptr || authTagLen != kTagBytes) {
            return AuthStatus::BadInput;
        }
        std::uint8_t msg[kMacHeaderBytes + kMaxArgsBytes];
        const std::size_t msgLen = buildMacMessage(
            kDomainTag, kDomainTagLen, vehicleId, cmdCounter, opcode, args, argLen, msg,
            sizeof(msg));
        if (msgLen == 0) {
            return AuthStatus::BadInput;
        }
        std::uint8_t expect[kTagBytes];
        hmacSha256(m_psk, m_pskLen, msg, msgLen, expect);
        if (!constTimeEqual(expect, authTag, kTagBytes)) {
            return AuthStatus::AuthFail;
        }
    }

    if (!m_store.writeLastCmdCounter(cmdCounter)) {
        return AuthStatus::StoreFail;
    }
    m_lastCmdCounter = cmdCounter;
    if (onForward != nullptr) {
        onForward(forwardCtx);
    }
    return AuthStatus::Ok;
}

}  // namespace UplinkAuth
