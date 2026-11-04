#ifndef UPLINKAUTH_HMACSHA256_HPP
#define UPLINKAUTH_HMACSHA256_HPP

#include <cstddef>
#include <cstdint>

namespace UplinkAuth {

// Portable SHA-256 / HMAC-SHA256 (FIPS 180-4 / RFC 2104). No OpenSSL.
// Flight path: no dynamic allocation.

class Sha256 {
  public:
    static constexpr std::size_t kDigestBytes = 32;
    static constexpr std::size_t kBlockBytes = 64;

    Sha256();
    void reset();
    void update(const std::uint8_t* data, std::size_t len);
    void final(std::uint8_t out[kDigestBytes]);

    static void hash(const std::uint8_t* data, std::size_t len,
                     std::uint8_t out[kDigestBytes]);

  private:
    void transform(const std::uint8_t block[kBlockBytes]);
    std::uint32_t m_state[8];
    std::uint64_t m_bitLen;
    std::uint8_t m_buf[kBlockBytes];
    std::size_t m_bufLen;
};

// HMAC-SHA256(key, msg) -> 32-byte tag.
void hmacSha256(const std::uint8_t* key, std::size_t keyLen,
                const std::uint8_t* msg, std::size_t msgLen,
                std::uint8_t out[Sha256::kDigestBytes]);

// Constant-time equality for digests / tags. Returns true iff equal.
bool constTimeEqual(const std::uint8_t* a, const std::uint8_t* b, std::size_t n);

}  // namespace UplinkAuth

#endif
