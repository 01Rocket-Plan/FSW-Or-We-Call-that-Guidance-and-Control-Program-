#ifndef NVFLAGS_NVFLAGSCORE_HPP
#define NVFLAGS_NVFLAGSCORE_HPP

#include "INvStore.hpp"

#include <cstddef>
#include <cstdint>

namespace NvFlags {

// Local return code mirroring Fw::Success style (no F´ dependency in core).
enum class CoreStatus { Success, Failure };

// Pure, host-testable core. No dynamic allocation on the flight path.
// Timestamps are injected by the caller; no wall clock is read here.
class NvFlagsCore {
  public:
    // Bits 0..3 are meaningful.
    static constexpr std::uint8_t kValidMask = 0x0Fu;

    // Dual-slot ping-pong record layout (little-endian on the wire).
    //   [0..3]  magic
    //   [4]     version
    //   [5]     flags mask (bits 0..3)
    //   [6]     latch bits: bit0 = sepCommanded, bit1 = dsChuteCommanded
    //   [7]     generation (U8, increments every commit; newest-slot pick)
    //   [8..11] phase (U32)
    //   [12..15] timestamp of last new-bit set (U32)
    //   [16..19] CRC32 over bytes [0..15]
    static constexpr std::size_t kRecordSize = 20;

    NvFlagsCore(INvStore& store, std::uint32_t phaseKeySeed = 0);

    // Load state from NVS. Picks the valid newer slot. Returns Success if a
    // valid record was loaded, Failure if none (state reset to defaults).
    CoreStatus init();

    // OR mask into current flags and commit immediately. On first successful
    // set of any new bit, records the injected timestamp.
    CoreStatus sepReleasedSet(std::uint8_t mask, std::uint32_t timestamp);

    // Returns RAM-cached mask.
    std::uint8_t sepReleasedGet() const { return m_flags; }

    // Set-only latches. Return Success if newly set, Failure if already set
    // (or on write failure).
    CoreStatus setSepCommanded(std::uint32_t timestamp);
    CoreStatus setDsChuteCommanded(std::uint32_t timestamp);
    bool sepCommanded() const { return m_latchA; }
    bool dsChuteCommanded() const { return m_latchB; }

    // Phase accessors.
    std::uint32_t phase() const { return m_phase; }
    CoreStatus setPhase(std::uint32_t phase, std::uint32_t timestamp);

    // Diagnostics.
    std::uint32_t writeCount() const { return m_writeCount; }
    std::uint32_t lastTimestamp() const { return m_lastTimestamp; }
    std::uint8_t activeSlot() const { return m_activeSlot; }

    // CRC helper exposed for tests.
    static std::uint32_t crc32(const std::uint8_t* data, std::size_t len);

  private:
    void encode(std::uint8_t slot, std::uint8_t* out) const;
    bool decode(const std::uint8_t* in, std::size_t len, std::uint8_t& flags,
                std::uint8_t& latches, std::uint32_t& phase,
                std::uint32_t& timestamp, std::uint8_t& generation) const;
    CoreStatus commit();

    INvStore& m_store;
    std::uint32_t m_phaseKeySeed;

    std::uint8_t m_flags = 0;
    bool m_latchA = false;
    bool m_latchB = false;
    std::uint32_t m_phase = 0;
    std::uint32_t m_lastTimestamp = 0;
    std::uint8_t m_generation = 0;

    // Start at 1 so the first commit writes slot 0.
    std::uint8_t m_activeSlot = 1;
    std::uint32_t m_writeCount = 0;
};

}  // namespace NvFlags

#endif
