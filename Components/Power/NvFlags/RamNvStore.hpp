#ifndef NVFLAGS_RAMNVSTORE_HPP
#define NVFLAGS_RAMNVSTORE_HPP

#include "INvStore.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace NvFlags {

//! RAM-backed fake store for host tests. Tracks write count; can fail writes.
class RamNvStore final : public INvStore {
  public:
    RamNvStore() = default;

    bool read(const char* key, uint8_t* buf, std::size_t maxLen,
              std::size_t& outLen) override {
        auto it = m_map.find(std::string(key));
        if (it == m_map.end()) {
            return false;
        }
        const std::vector<uint8_t>& v = it->second;
        if (v.size() > maxLen) {
            return false;
        }
        for (std::size_t i = 0; i < v.size(); ++i) {
            buf[i] = v[i];
        }
        outLen = v.size();
        return true;
    }

    bool write(const char* key, const uint8_t* buf, std::size_t len) override {
        if (m_failWrites) {
            return false;
        }
        ++m_writeCount;
        m_map[std::string(key)] = std::vector<uint8_t>(buf, buf + len);
        return true;
    }

    bool erase(const char* key) override {
        m_map.erase(std::string(key));
        return true;
    }

    void setFailWrites(bool fail) { m_failWrites = fail; }
    std::uint32_t writeCount() const { return m_writeCount; }
    void resetWriteCount() { m_writeCount = 0; }
    bool contains(const char* key) const {
        return m_map.find(std::string(key)) != m_map.end();
    }
    void clear() {
        m_map.clear();
        m_writeCount = 0;
        m_failWrites = false;
    }

  private:
    std::map<std::string, std::vector<uint8_t>> m_map;
    std::uint32_t m_writeCount = 0;
    bool m_failWrites = false;
};

}  // namespace NvFlags

#endif
