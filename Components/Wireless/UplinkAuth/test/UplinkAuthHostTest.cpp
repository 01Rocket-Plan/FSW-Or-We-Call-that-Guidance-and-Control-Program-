#include "HmacSha256.hpp"
#include "OpcodeAuthTable.hpp"
#include "RamAuthNvStore.hpp"
#include "UplinkAuthCore.hpp"

#include <cstdio>
#include <cstring>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("CHECK failed: %s (%s:%d)\n", #cond, __FILE__,         \
                        __LINE__);                                             \
            ++g_failures;                                                      \
        }                                                                      \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        const auto _a = (a);                                                   \
        const auto _b = (b);                                                   \
        if (!(_a == _b)) {                                                     \
            std::printf(                                                       \
                "CHECK_EQ failed: %s == %s (%s:%d) got=%u expect=%u\n", #a,    \
                #b, __FILE__, __LINE__, (unsigned)_a, (unsigned)_b);           \
            ++g_failures;                                                      \
        }                                                                      \
    } while (0)

using UplinkAuth::AuthStatus;
using UplinkAuth::hmacSha256;
using UplinkAuth::OpcodeAuthTable;
using UplinkAuth::RamAuthNvStore;
using UplinkAuth::Sha256;
using UplinkAuth::UplinkAuthCore;

// FAKE test-only PSK — clearly labeled, never a flight key. Loaded via NVS fake.
const std::uint8_t kFakePsk[] = {
    'F', 'A', 'K', 'E', '_', 'T', 'E', 'S', 'T', '_', 'P', 'S', 'K', '_',
    'D', 'O', '_', 'N', 'O', 'T', '_', 'U', 'S', 'E', '_', 'I', 'N', '_',
    'F', 'L', 'I', 'G', 'H', 'T', '!', '!'};
constexpr std::size_t kFakePskLen = sizeof(kFakePsk);

constexpr std::uint16_t kAbort = OpcodeAuthTable::kProvisionalAbortOpcode;
constexpr std::uint16_t kGimbal = OpcodeAuthTable::kProvisionalGimbalOpcode;

void makeTagWithDomain(const char* domain, std::size_t domainLen, std::uint8_t vehicleId,
                       std::uint32_t cmdCounter, std::uint16_t opcode,
                       const std::uint8_t* args, std::uint16_t argLen,
                       std::uint8_t out[UplinkAuthCore::kTagBytes]) {
    std::uint8_t msg[UplinkAuthCore::kMacHeaderBytes + UplinkAuthCore::kMaxArgsBytes];
    const std::size_t n = UplinkAuthCore::buildMacMessage(
        domain, domainLen, vehicleId, cmdCounter, opcode, args, argLen, msg, sizeof(msg));
    CHECK(n != 0u);
    hmacSha256(kFakePsk, kFakePskLen, msg, n, out);
}

void makeTag(std::uint8_t vehicleId, std::uint32_t cmdCounter, std::uint16_t opcode,
             const std::uint8_t* args, std::uint16_t argLen,
             std::uint8_t out[UplinkAuthCore::kTagBytes]) {
    makeTagWithDomain(UplinkAuthCore::kDomainTag, UplinkAuthCore::kDomainTagLen, vehicleId,
                      cmdCounter, opcode, args, argLen, out);
}

struct OrderProbe {
    std::uint32_t writeCountAtForward = 0;
    bool forwarded = false;
    RamAuthNvStore* store = nullptr;
};

void onForwardProbe(void* ctx) {
    OrderProbe* p = static_cast<OrderProbe*>(ctx);
    p->forwarded = true;
    p->writeCountAtForward = p->store->writeCount();
}

void testSha256EmptyKnownVector() {
    std::uint8_t dig[32];
    Sha256::hash(reinterpret_cast<const std::uint8_t*>(""), 0, dig);
    const char* expectHex =
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
    char got[65];
    for (int i = 0; i < 32; ++i) {
        std::snprintf(got + i * 2, 3, "%02x", dig[i]);
    }
    CHECK(std::strcmp(got, expectHex) == 0);
}

void testHmacRfc4231Case1() {
    const std::uint8_t key[20] = {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
                                  0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
                                  0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b};
    const std::uint8_t data[8] = {'H', 'i', ' ', 'T', 'h', 'e', 'r', 'e'};
    std::uint8_t out[32];
    hmacSha256(key, sizeof(key), data, sizeof(data), out);
    const char* expectHex =
        "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7";
    char got[65];
    for (int i = 0; i < 32; ++i) {
        std::snprintf(got + i * 2, 3, "%02x", out[i]);
    }
    CHECK(std::strcmp(got, expectHex) == 0);
}

void testReplayRejected() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    const std::uint8_t args[] = {0xAA};
    std::uint8_t tag[32];
    makeTag(0x01, 1, kAbort, args, 1, tag);
    CHECK(core.accept(1, kAbort, 0x01, args, 1, tag, 32, nullptr, nullptr) == AuthStatus::Ok);
    CHECK(core.accept(1, kAbort, 0x01, args, 1, tag, 32, nullptr, nullptr) == AuthStatus::Replay);
}

void testHmacOkAndFail() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    const std::uint8_t args[] = {0x01, 0x02};
    std::uint8_t tag[32];
    makeTag(0x02, 10, kAbort, args, 2, tag);
    CHECK(core.accept(10, kAbort, 0x02, args, 2, tag, 32, nullptr, nullptr) == AuthStatus::Ok);
    makeTag(0x02, 11, kAbort, args, 2, tag);
    tag[0] ^= 0xFFu;
    CHECK(core.accept(11, kAbort, 0x02, args, 2, tag, 32, nullptr, nullptr) == AuthStatus::AuthFail);
}

void testTamperedArgsRejected() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    const std::uint8_t argsGood[] = {0x10, 0x20, 0x30};
    const std::uint8_t argsBad[] = {0x10, 0x20, 0x31};
    std::uint8_t tag[32];
    makeTag(0x01, 3, kAbort, argsGood, 3, tag);
    CHECK(core.accept(3, kAbort, 0x01, argsBad, 3, tag, 32, nullptr, nullptr) == AuthStatus::AuthFail);
}

void testWrongDomainRejected() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    const std::uint8_t args[] = {0x42};
    std::uint8_t tag[32];
    makeTagWithDomain("01RP-UPL-v0", 11, 0x01, 4, kAbort, args, 1, tag);
    CHECK(core.accept(4, kAbort, 0x01, args, 1, tag, 32, nullptr, nullptr) == AuthStatus::AuthFail);
}

void testArgLenMismatchRejected() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    const std::uint8_t args2[] = {0x01, 0x02};
    const std::uint8_t args3[] = {0x01, 0x02, 0x00};
    std::uint8_t tag[32];
    makeTag(0x01, 5, kAbort, args2, 2, tag);
    CHECK(core.accept(5, kAbort, 0x01, args3, 3, tag, 32, nullptr, nullptr) == AuthStatus::AuthFail);
    CHECK(core.accept(6, kAbort, 0x01, nullptr, 2, tag, 32, nullptr, nullptr) == AuthStatus::BadInput);
}

void testForbiddenCounterRejected() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    CHECK(core.accept(UplinkAuthCore::kCmdCounterForbidden, kAbort, 0x01, nullptr, 0, nullptr, 0,
                      nullptr, nullptr) == AuthStatus::CounterInvalid);
}

void testPersistBeforeForward() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    OrderProbe probe;
    probe.store = &store;
    const std::uint8_t args[] = {0x7E};
    std::uint8_t tag[32];
    makeTag(0x01, 7, kAbort, args, 1, tag);
    CHECK(core.accept(7, kAbort, 0x01, args, 1, tag, 32, onForwardProbe, &probe) == AuthStatus::Ok);
    CHECK(probe.forwarded);
    CHECK(probe.writeCountAtForward >= 1u);
}

void testStoreFailDoesNotForward() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    OrderProbe probe;
    probe.store = &store;
    store.setFailWrites(true);
    const std::uint8_t args[] = {0x01};
    std::uint8_t tag[32];
    makeTag(0x01, 8, kAbort, args, 1, tag);
    CHECK(core.accept(8, kAbort, 0x01, args, 1, tag, 32, onForwardProbe, &probe) == AuthStatus::StoreFail);
    CHECK(!probe.forwarded);
}

void testUnknownOpcodeWithoutTagRejected() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    constexpr std::uint16_t kUnknown = 0x00A0;
    CHECK(core.hmacExemptTable().requiresHmac(kUnknown));
    CHECK(core.accept(5, kUnknown, 0x01, nullptr, 0, nullptr, 0, nullptr, nullptr) ==
          AuthStatus::BadInput);
}

void testGimbalOpcodeWithoutTagAcceptedAtAuthLayer() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    CHECK(core.hmacExemptTable().isExempt(kGimbal));
    CHECK(!core.hmacExemptTable().requiresHmac(kGimbal));
    // Auth layer accepts without tag; §8 whitelist / param-range still required downstream.
    CHECK(core.accept(5, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) ==
          AuthStatus::Ok);
}

void testOverrideClassWrongTagRejected() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    CHECK(core.hmacExemptTable().requiresHmac(kAbort));
    const std::uint8_t args[] = {0x01};
    std::uint8_t tag[32];
    makeTag(0x01, 9, kAbort, args, 1, tag);
    tag[0] ^= 0xFFu;
    CHECK(core.accept(9, kAbort, 0x01, args, 1, tag, 32, nullptr, nullptr) == AuthStatus::AuthFail);
}

void testMissingKeyWhenHmacRequired() {
    RamAuthNvStore store;
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    CHECK(!core.pskLoaded());
    std::uint8_t tag[32];
    std::memset(tag, 0, sizeof(tag));
    CHECK(core.accept(1, kAbort, 0x01, nullptr, 0, tag, 32, nullptr, nullptr) ==
          AuthStatus::MissingKey);
}

void testPersistedCounterSurvivesReinit() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    {
        UplinkAuthCore core(store);
        CHECK(core.init() == AuthStatus::Ok);
        std::uint8_t tag[32];
        makeTag(0x01, 42, kAbort, nullptr, 0, tag);
        CHECK(core.accept(42, kAbort, 0x01, nullptr, 0, tag, 32, nullptr, nullptr) ==
              AuthStatus::Ok);
    }
    UplinkAuthCore core2(store);
    CHECK(core2.init() == AuthStatus::Ok);
    CHECK_EQ(core2.lastCmdCounter(), 42u);
}

// PR #13 review (Edward): an unsigned (exempt) packet must not advance the signed
// counter — otherwise one gimbal packet with a huge counter locks out abort.
void testExemptDoesNotAdvanceSignedCounter() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    const std::uint32_t writesBefore = store.writeCount();
    CHECK(core.accept(0xFFFFFFFEu, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) ==
          AuthStatus::Ok);
    CHECK_EQ(core.lastCmdCounter(), 0u);
    CHECK_EQ(store.writeCount(), writesBefore);  // exempt path never writes NVS
    std::uint8_t tag[32];
    makeTag(0x01, 1, kAbort, nullptr, 0, tag);
    CHECK(core.accept(1, kAbort, 0x01, nullptr, 0, tag, 32, nullptr, nullptr) == AuthStatus::Ok);
    CHECK_EQ(core.lastCmdCounter(), 1u);
}

void testSignedDoesNotBlockExempt() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    std::uint8_t tag[32];
    makeTag(0x01, 100, kAbort, nullptr, 0, tag);
    CHECK(core.accept(100, kAbort, 0x01, nullptr, 0, tag, 32, nullptr, nullptr) == AuthStatus::Ok);
    CHECK(core.accept(1, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) ==
          AuthStatus::Ok);
    CHECK_EQ(core.lastExemptCounter(), 1u);
    CHECK_EQ(core.lastCmdCounter(), 100u);
}

void testExemptCounterOwnReplayWindow() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    CHECK(core.accept(5, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) == AuthStatus::Ok);
    CHECK(core.accept(5, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) ==
          AuthStatus::Replay);
    CHECK(core.accept(4, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) ==
          AuthStatus::Replay);
    CHECK(core.accept(6, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) == AuthStatus::Ok);
    CHECK(core.accept(UplinkAuthCore::kCmdCounterForbidden, kGimbal, 0x02, nullptr, 0, nullptr, 0,
                      nullptr, nullptr) == AuthStatus::CounterInvalid);
}

void testExemptCounterNotPersisted() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    {
        UplinkAuthCore core(store);
        CHECK(core.init() == AuthStatus::Ok);
        CHECK(core.accept(77, kGimbal, 0x02, nullptr, 0, nullptr, 0, nullptr, nullptr) ==
              AuthStatus::Ok);
    }
    UplinkAuthCore core2(store);
    CHECK(core2.init() == AuthStatus::Ok);
    CHECK_EQ(core2.lastExemptCounter(), 0u);
    CHECK_EQ(core2.lastCmdCounter(), 0u);
}

void testBadTagLength() {
    RamAuthNvStore store;
    store.setPsk(kFakePsk, kFakePskLen);
    UplinkAuthCore core(store);
    CHECK(core.init() == AuthStatus::Ok);
    std::uint8_t tag[16];
    std::memset(tag, 0, sizeof(tag));
    CHECK(core.accept(1, kAbort, 0x01, nullptr, 0, tag, 16, nullptr, nullptr) ==
          AuthStatus::BadInput);
}

void testDefaultExemptTableIsGimbalOnly() {
    OpcodeAuthTable t = OpcodeAuthTable::makeDefault();
    CHECK_EQ(t.count, 1u);
    CHECK(t.isExempt(OpcodeAuthTable::kProvisionalGimbalOpcode));
    CHECK(!t.isExempt(OpcodeAuthTable::kProvisionalAbortOpcode));
    CHECK(t.requiresHmac(OpcodeAuthTable::kProvisionalAbortOpcode));
    CHECK(t.requiresHmac(0x00FF));  // unknown always requires HMAC
}

}  // namespace

int main() {
    testSha256EmptyKnownVector();
    testHmacRfc4231Case1();
    testReplayRejected();
    testHmacOkAndFail();
    testTamperedArgsRejected();
    testWrongDomainRejected();
    testArgLenMismatchRejected();
    testForbiddenCounterRejected();
    testPersistBeforeForward();
    testStoreFailDoesNotForward();
    testUnknownOpcodeWithoutTagRejected();
    testGimbalOpcodeWithoutTagAcceptedAtAuthLayer();
    testOverrideClassWrongTagRejected();
    testMissingKeyWhenHmacRequired();
    testPersistedCounterSurvivesReinit();
    testBadTagLength();
    testExemptDoesNotAdvanceSignedCounter();
    testSignedDoesNotBlockExempt();
    testExemptCounterOwnReplayWindow();
    testExemptCounterNotPersisted();
    testDefaultExemptTableIsGimbalOnly();
    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d CHECK(s) FAILED\n", g_failures);
    return 1;
}
