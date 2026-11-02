#ifndef NVFLAGS_INVSTORE_HPP
#define NVFLAGS_INVSTORE_HPP

#include <cstddef>
#include <cstdint>

namespace NvFlags {

//! Abstract non-volatile key/value blob store (ESP32 NVS or RAM fake).
class INvStore {
  public:
    virtual ~INvStore() {}

    //! Read key into buf. Returns false if missing or error.
    virtual bool read(const char* key, uint8_t* buf, std::size_t maxLen,
                      std::size_t& outLen) = 0;

    //! Write key. Returns false on error.
    virtual bool write(const char* key, const uint8_t* buf, std::size_t len) = 0;

    //! Erase key. Returns false on error.
    virtual bool erase(const char* key) = 0;
};

}  // namespace NvFlags

#endif
