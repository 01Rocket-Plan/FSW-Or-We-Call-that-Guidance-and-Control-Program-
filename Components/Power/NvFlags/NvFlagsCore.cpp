#include "NvFlagsCore.hpp"

#include <cstring>

namespace NvFlags {

namespace {
constexpr std::uint32_t kMagic = 0x4E56464Cu;  // "NVFL"
constexpr std::uint8_t kVersion = 1;
constexpr const char* kSlotKeys[2] = {"nvf_slot0", "nvf_slot1"};
}  // namespace

NvFlagsCore::NvFlagsCore(INvStore& store, std::uint32_t phaseKeySeed)
    : m_store(store), m_phaseKeySeed(phaseKeySeed) {}

std::uint32_t NvFlagsCore::crc32(const std::uint8_t* data, std::size_t len) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            const std::uint32_t mask = (crc & 1u) ? 0xEDB88320u : 0u;
            crc = (crc >> 1) ^ mask;
        }
    }
    return ~crc;
}

void NvFlagsCore::encode(std::uint8_t slot, std::uint8_t* out) const {
    (void)slot;
    std::uint32_t magic = kMagic;
    std::memcpy(&out[0], &magic, 4);
    out[4] = kVersion;
    out[5] = static_cast<std::uint8_t>(m_flags & kValidMask);
    std::uint8_t latches = 0;
    if (m_latchA) latches |= 0x01u;
    if (m_latchB) latches |= 0x02u;
    out[6] = latches;
    out[7] = m_generation;
    std::memcpy(&out[8], &m_phase, 4);
    std::memcpy(&out[12], &m_lastTimestamp, 4);
    const std::uint32_t crc = crc32(out, 16);
    std::memcpy(&out[16], &crc, 4);
}

bool NvFlagsCore::decode(const std::uint8_t* in, std::size_t len,
                         std::uint8_t& flags, std::uint8_t& latches,
                         std::uint32_t& phase, std::uint32_t& timestamp,
                         std::uint8_t& generation) const {
    if (len != kRecordSize) {
        return false;
    }
    std::uint32_t magic = 0;
    std::memcpy(&magic, &in[0], 4);
    if (magic != kMagic) {
        return false;
    }
    if (in[4] != kVersion) {
        return false;
    }
    std::uint32_t crcStored = 0;
    std::memcpy(&crcStored, &in[16], 4);
    const std::uint32_t crcCalc = crc32(in, 16);
    if (crcStored != crcCalc) {
        return false;
    }
    flags = static_cast<std::uint8_t>(in[5] & kValidMask);
    latches = in[6];
    generation = in[7];
    std::memcpy(&phase, &in[8], 4);
    std::memcpy(&timestamp, &in[12], 4);
    return true;
}

CoreStatus NvFlagsCore::init() {
    bool have[2] = {false, false};
    std::uint8_t flags[2] = {0, 0};
    std::uint8_t latches[2] = {0, 0};
    std::uint32_t phase[2] = {0, 0};
    std::uint32_t ts[2] = {0, 0};
    std::uint8_t gen[2] = {0, 0};

    for (std::uint8_t s = 0; s < 2; ++s) {
        std::uint8_t buf[kRecordSize];
        std::size_t len = 0;
        if (m_store.read(kSlotKeys[s], buf, sizeof(buf), len)) {
            have[s] = decode(buf, len, flags[s], latches[s], phase[s], ts[s], gen[s]);
        }
    }

    int chosen = -1;
    if (have[0] && have[1]) {
        // Newest = higher generation (wrap-aware via signed delta), then
        // higher phase, then higher timestamp, then higher slot index.
        const std::int8_t dgen =
            static_cast<std::int8_t>(gen[1] - gen[0]);
        if (dgen > 0) {
            chosen = 1;
        } else if (dgen < 0) {
            chosen = 0;
        } else if (phase[1] > phase[0]) {
            chosen = 1;
        } else if (phase[1] < phase[0]) {
            chosen = 0;
        } else if (ts[1] > ts[0]) {
            chosen = 1;
        } else {
            chosen = 0;
        }
    } else if (have[0]) {
        chosen = 0;
    } else if (have[1]) {
        chosen = 1;
    }

    if (chosen < 0) {
        m_flags = 0;
        m_latchA = false;
        m_latchB = false;
        m_phase = 0;
        m_lastTimestamp = 0;
        m_generation = 0;
        // Start at 1 so the first commit writes slot 0.
        m_activeSlot = 1;
        return CoreStatus::Failure;
    }

    m_flags = flags[chosen];
    m_latchA = (latches[chosen] & 0x01u) != 0;
    m_latchB = (latches[chosen] & 0x02u) != 0;
    m_phase = phase[chosen];
    m_lastTimestamp = ts[chosen];
    m_generation = gen[chosen];
    m_activeSlot = static_cast<std::uint8_t>(chosen);
    return CoreStatus::Success;
}

CoreStatus NvFlagsCore::commit() {
    // Ping-pong: write to the inactive slot, then flip active.
    const std::uint8_t next = static_cast<std::uint8_t>(m_activeSlot ^ 1u);
    const std::uint8_t prevGen = m_generation;
    m_generation = static_cast<std::uint8_t>(m_generation + 1u);
    std::uint8_t buf[kRecordSize];
    encode(next, buf);
    if (!m_store.write(kSlotKeys[next], buf, sizeof(buf))) {
        m_generation = prevGen;
        return CoreStatus::Failure;
    }
    m_activeSlot = next;
    ++m_writeCount;
    return CoreStatus::Success;
}

CoreStatus NvFlagsCore::sepReleasedSet(std::uint8_t mask, std::uint32_t timestamp) {
    const std::uint8_t newBits =
        static_cast<std::uint8_t>((mask & kValidMask) & ~m_flags);
    if (newBits == 0) {
        // No change: skip write to reduce wear.
        return CoreStatus::Success;
    }
    const std::uint8_t prevFlags = m_flags;
    const std::uint32_t prevTs = m_lastTimestamp;
    m_flags = static_cast<std::uint8_t>(m_flags | newBits);
    // Record timestamp of this new-bit set before encoding.
    m_lastTimestamp = timestamp;
    const CoreStatus st = commit();
    if (st != CoreStatus::Success) {
        m_flags = prevFlags;
        m_lastTimestamp = prevTs;
        return CoreStatus::Failure;
    }
    return CoreStatus::Success;
}

CoreStatus NvFlagsCore::setSepCommanded(std::uint32_t timestamp) {
    (void)timestamp;
    if (m_latchA) {
        return CoreStatus::Failure;
    }
    m_latchA = true;
    const CoreStatus st = commit();
    if (st != CoreStatus::Success) {
        m_latchA = false;
        return CoreStatus::Failure;
    }
    return CoreStatus::Success;
}

CoreStatus NvFlagsCore::setDsChuteCommanded(std::uint32_t timestamp) {
    (void)timestamp;
    if (m_latchB) {
        return CoreStatus::Failure;
    }
    m_latchB = true;
    const CoreStatus st = commit();
    if (st != CoreStatus::Success) {
        m_latchB = false;
        return CoreStatus::Failure;
    }
    return CoreStatus::Success;
}

CoreStatus NvFlagsCore::setPhase(std::uint32_t phase, std::uint32_t timestamp) {
    (void)timestamp;
    if (phase == m_phase) {
        return CoreStatus::Success;  // skip identical write
    }
    const std::uint32_t prev = m_phase;
    m_phase = phase;
    const CoreStatus st = commit();
    if (st != CoreStatus::Success) {
        m_phase = prev;
        return CoreStatus::Failure;
    }
    return CoreStatus::Success;
}

}  // namespace NvFlags
