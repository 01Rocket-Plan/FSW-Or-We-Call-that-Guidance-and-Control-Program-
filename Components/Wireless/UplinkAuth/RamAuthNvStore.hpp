#ifndef UPLINKAUTH_RAMAUTHNVSTORE_HPP
#define UPLINKAUTH_RAMAUTHNVSTORE_HPP

#include "IAuthNvStore.hpp"

#include <cstring>
#include <vector>

namespace UplinkAuth {

//! RAM-backed store for host unit tests only.
class RamAuthNvStore final : public IAuthNvStore {
  public:
    RamAuthNvStore() = default;

    bool readLastCmdCounter(std::uint32_t& out) override {
        if (!m_haveCounter) {
            return false;
        }
        out = m_lastCounter;
        return true;
    }

    bool writeLastCmdCounter(std::uint32_t value) override {
        if (m_failWrites) {
            return false;
        }
        m_lastCounter = value;
        m_haveCounter = true;
        ++m_writeCount;
        return true;
    }

    bool readPsk(std::uint8_t* buf, std::size_t maxLen, std::size_t& outLen) override {
        if (!m_havePsk) {
            return false;
        }
        if (m_psk.size() > maxLen) {
            return false;
        }
        std::memcpy(buf, m_psk.data(), m_psk.size());
        outLen = m_psk.size();
        return true;
    }

    void setPsk(const std::uint8_t* psk, std::size_t len) {
        m_psk.assign(psk, psk + len);
        m_havePsk = true;
    }

    void clearPsk() {
        m_psk.clear();
        m_havePsk = false;
    }

    void setFailWrites(bool fail) { m_failWrites = fail; }
    std::uint32_t writeCount() const { return m_writeCount; }
    void resetWriteCount() { m_writeCount = 0; }

  private:
    std::vector<std::uint8_t> m_psk;
    bool m_havePsk = false;
    std::uint32_t m_lastCounter = 0;
    bool m_haveCounter = false;
    bool m_failWrites = false;
    std::uint32_t m_writeCount = 0;
};

}  // namespace UplinkAuth

#endif
