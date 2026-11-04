#ifndef UPLINKAUTH_IAUTHNVSTORE_HPP
#define UPLINKAUTH_IAUTHNVSTORE_HPP

#include <cstddef>
#include <cstdint>

namespace UplinkAuth {

//! Abstract NVS access for uplink auth (PSK blob + last accepted cmdCounter).
//! Flight wiring uses nvFlags / platform NVS; host tests use RamAuthNvStore.
class IAuthNvStore {
  public:
    virtual ~IAuthNvStore() {}

    //! Read last accepted cmdCounter. Returns false if missing (treat as 0).
    virtual bool readLastCmdCounter(std::uint32_t& out) = 0;

    //! Persist last accepted cmdCounter after a successful accept.
    virtual bool writeLastCmdCounter(std::uint32_t value) = 0;

    //! Read PSK bytes into buf. outLen set to actual length. Returns false if missing.
    virtual bool readPsk(std::uint8_t* buf, std::size_t maxLen, std::size_t& outLen) = 0;
};

}  // namespace UplinkAuth

#endif
