#include "NvFlagsCore.hpp"
#include "RamNvStore.hpp"

#include <cstdio>
#include <cstring>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("CHECK failed: %s (%s:%d)\n", #cond, __FILE__,    \
                        __LINE__);                                        \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

#define CHECK_EQ(a, b)                                                    \
    do {                                                                  \
        const auto _a = (a);                                              \
        const auto _b = (b);                                              \
        if (!(_a == _b)) {                                                \
            std::printf("CHECK_EQ failed: %s == %s (%s:%d) got=%u expect=%u\n", \
                        #a, #b, __FILE__, __LINE__,                       \
                        (unsigned)_a, (unsigned)_b);                      \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

using NvFlags::CoreStatus;
using NvFlags::NvFlagsCore;
using NvFlags::RamNvStore;

void testRepeatedIdenticalSetNoWrite() {
    RamNvStore store;
    NvFlagsCore core(store);
    (void)core.init();

    CHECK(core.sepReleasedSet(0x01, 100) == CoreStatus::Success);
    const std::uint32_t afterFirst = store.writeCount();
    CHECK_EQ(afterFirst, 1u);

    for (int i = 0; i < 5; ++i) {
        CHECK(core.sepReleasedSet(0x01, 200 + i) == CoreStatus::Success);
    }
    CHECK_EQ(store.writeCount(), afterFirst);
    CHECK_EQ(core.sepReleasedGet(), 0x01u);
    CHECK_EQ(core.lastTimestamp(), 100u);
}

void testFirstNewBitRecordsTimestamp() {
    RamNvStore store;
    NvFlagsCore core(store);
    (void)core.init();

    CHECK(core.sepReleasedSet(0x01, 111) == CoreStatus::Success);
    CHECK_EQ(core.lastTimestamp(), 111u);

    CHECK(core.sepReleasedSet(0x01, 222) == CoreStatus::Success);
    CHECK_EQ(core.lastTimestamp(), 111u);

    CHECK(core.sepReleasedSet(0x02, 333) == CoreStatus::Success);
    CHECK_EQ(core.lastTimestamp(), 333u);
    CHECK_EQ(core.sepReleasedGet(), 0x03u);
}

void testRecoveryFromCorruptedSlot() {
    RamNvStore store;
    {
        NvFlagsCore core(store);
        (void)core.init();
        CHECK(core.sepReleasedSet(0x01, 10) == CoreStatus::Success);
        CHECK(core.sepReleasedSet(0x02, 20) == CoreStatus::Success);
        CHECK(core.setPhase(7, 30) == CoreStatus::Success);
        CHECK_EQ(core.activeSlot(), 0u);
    }
    std::uint8_t buf[64];
    std::size_t len = 0;
    CHECK(store.read("nvf_slot0", buf, sizeof(buf), len));
    CHECK(len == NvFlagsCore::kRecordSize);
    buf[5] ^= 0xFFu;
    CHECK(store.write("nvf_slot0", buf, len));

    NvFlagsCore core2(store);
    CHECK(core2.init() == CoreStatus::Success);
    CHECK_EQ(core2.sepReleasedGet(), 0x03u);
    CHECK_EQ(core2.phase(), 0u);
}

void testCrcErrorHandling() {
    RamNvStore store;
    {
        NvFlagsCore core(store);
        (void)core.init();
        CHECK(core.sepReleasedSet(0x0F, 1) == CoreStatus::Success);
        CHECK_EQ(core.activeSlot(), 0u);
    }
    std::uint8_t buf[64];
    std::size_t len = 0;
    CHECK(store.read("nvf_slot0", buf, sizeof(buf), len));
    CHECK(len == NvFlagsCore::kRecordSize);
    buf[16] ^= 0x01u;
    CHECK(store.write("nvf_slot0", buf, len));

    NvFlagsCore core2(store);
    CHECK(core2.init() == CoreStatus::Failure);
    CHECK_EQ(core2.sepReleasedGet(), 0u);
}

void testWriteFailureReturnsFailure() {
    RamNvStore store;
    NvFlagsCore core(store);
    (void)core.init();

    store.setFailWrites(true);
    CHECK(core.sepReleasedSet(0x01, 5) == CoreStatus::Failure);
    CHECK_EQ(core.sepReleasedGet(), 0u);

    store.setFailWrites(false);
    CHECK(core.sepReleasedSet(0x01, 6) == CoreStatus::Success);
    CHECK_EQ(core.sepReleasedGet(), 0x01u);
}

void testLatchesSetOnly() {
    RamNvStore store;
    NvFlagsCore core(store);
    (void)core.init();

    CHECK(core.setSepCommanded(1) == CoreStatus::Success);
    CHECK(core.sepCommanded());
    CHECK(core.setSepCommanded(2) == CoreStatus::Failure);
    CHECK(core.sepCommanded());

    CHECK(core.setDsChuteCommanded(3) == CoreStatus::Success);
    CHECK(core.dsChuteCommanded());

    NvFlagsCore core2(store);
    CHECK(core2.init() == CoreStatus::Success);
    CHECK(core2.sepCommanded());
    CHECK(core2.dsChuteCommanded());
}

void testPhaseSkipIdentical() {
    RamNvStore store;
    NvFlagsCore core(store);
    (void)core.init();

    CHECK(core.setPhase(3, 1) == CoreStatus::Success);
    const std::uint32_t w = store.writeCount();
    CHECK(core.setPhase(3, 2) == CoreStatus::Success);
    CHECK_EQ(store.writeCount(), w);
    CHECK(core.setPhase(4, 3) == CoreStatus::Success);
    CHECK_EQ(store.writeCount(), w + 1);
}

void testNewerSlotWins() {
    RamNvStore store;
    {
        NvFlagsCore core(store);
        (void)core.init();
        CHECK(core.setPhase(1, 1) == CoreStatus::Success);
        CHECK(core.setPhase(2, 2) == CoreStatus::Success);
    }
    NvFlagsCore core2(store);
    CHECK(core2.init() == CoreStatus::Success);
    CHECK_EQ(core2.phase(), 2u);
}

}  // namespace

int main() {
    testRepeatedIdenticalSetNoWrite();
    testFirstNewBitRecordsTimestamp();
    testRecoveryFromCorruptedSlot();
    testCrcErrorHandling();
    testWriteFailureReturnsFailure();
    testLatchesSetOnly();
    testPhaseSkipIdentical();
    testNewerSlotWins();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d CHECK(s) FAILED\n", g_failures);
    return 1;
}
