#include "HmacSha256.hpp"

#include <cstring>

namespace UplinkAuth {

namespace {

inline std::uint32_t rotr(std::uint32_t x, std::uint32_t n) {
    return (x >> n) | (x << (32u - n));
}

inline std::uint32_t be32Load(const std::uint8_t* p) {
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) |
           (std::uint32_t(p[2]) << 8) | std::uint32_t(p[3]);
}

inline void be32Store(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>((v >> 24) & 0xFFu);
    p[1] = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
    p[2] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
    p[3] = static_cast<std::uint8_t>(v & 0xFFu);
}

const std::uint32_t kK[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u,
    0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u,
    0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
    0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
    0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au,
    0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};

}  // namespace

Sha256::Sha256() { reset(); }

void Sha256::reset() {
    m_state[0] = 0x6a09e667u;
    m_state[1] = 0xbb67ae85u;
    m_state[2] = 0x3c6ef372u;
    m_state[3] = 0xa54ff53au;
    m_state[4] = 0x510e527fu;
    m_state[5] = 0x9b05688cu;
    m_state[6] = 0x1f83d9abu;
    m_state[7] = 0x5be0cd19u;
    m_bitLen = 0;
    m_bufLen = 0;
}

void Sha256::transform(const std::uint8_t block[kBlockBytes]) {
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
        w[i] = be32Load(block + static_cast<std::size_t>(i) * 4u);
    }
    for (int i = 16; i < 64; ++i) {
        const std::uint32_t s0 =
            rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const std::uint32_t s1 =
            rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    std::uint32_t a = m_state[0];
    std::uint32_t b = m_state[1];
    std::uint32_t c = m_state[2];
    std::uint32_t d = m_state[3];
    std::uint32_t e = m_state[4];
    std::uint32_t f = m_state[5];
    std::uint32_t g = m_state[6];
    std::uint32_t h = m_state[7];

    for (int i = 0; i < 64; ++i) {
        const std::uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        const std::uint32_t ch = (e & f) ^ ((~e) & g);
        const std::uint32_t temp1 = h + S1 + ch + kK[i] + w[i];
        const std::uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t temp2 = S0 + maj;
        h = g;
        g = f;
        f = e;
        e = d + temp1;
        d = c;
        c = b;
        b = a;
        a = temp1 + temp2;
    }

    m_state[0] += a;
    m_state[1] += b;
    m_state[2] += c;
    m_state[3] += d;
    m_state[4] += e;
    m_state[5] += f;
    m_state[6] += g;
    m_state[7] += h;
}

void Sha256::update(const std::uint8_t* data, std::size_t len) {
    for (std::size_t i = 0; i < len; ++i) {
        m_buf[m_bufLen++] = data[i];
        if (m_bufLen == kBlockBytes) {
            transform(m_buf);
            m_bitLen += 512u;
            m_bufLen = 0;
        }
    }
}

void Sha256::final(std::uint8_t out[kDigestBytes]) {
    const std::uint64_t totalBits = m_bitLen + static_cast<std::uint64_t>(m_bufLen) * 8u;
    m_buf[m_bufLen++] = 0x80u;
    if (m_bufLen > 56) {
        while (m_bufLen < kBlockBytes) {
            m_buf[m_bufLen++] = 0;
        }
        transform(m_buf);
        m_bufLen = 0;
    }
    while (m_bufLen < 56) {
        m_buf[m_bufLen++] = 0;
    }
    for (int i = 7; i >= 0; --i) {
        m_buf[m_bufLen++] = static_cast<std::uint8_t>((totalBits >> (8 * i)) & 0xFFu);
    }
    transform(m_buf);
    for (int i = 0; i < 8; ++i) {
        be32Store(out + static_cast<std::size_t>(i) * 4u, m_state[i]);
    }
}

void Sha256::hash(const std::uint8_t* data, std::size_t len,
                  std::uint8_t out[kDigestBytes]) {
    Sha256 ctx;
    ctx.update(data, len);
    ctx.final(out);
}

void hmacSha256(const std::uint8_t* key, std::size_t keyLen,
                const std::uint8_t* msg, std::size_t msgLen,
                std::uint8_t out[Sha256::kDigestBytes]) {
    std::uint8_t keyBlock[Sha256::kBlockBytes];
    std::memset(keyBlock, 0, sizeof(keyBlock));
    if (keyLen > Sha256::kBlockBytes) {
        Sha256::hash(key, keyLen, keyBlock);
    } else {
        std::memcpy(keyBlock, key, keyLen);
    }

    std::uint8_t ipad[Sha256::kBlockBytes];
    std::uint8_t opad[Sha256::kBlockBytes];
    for (std::size_t i = 0; i < Sha256::kBlockBytes; ++i) {
        ipad[i] = static_cast<std::uint8_t>(keyBlock[i] ^ 0x36u);
        opad[i] = static_cast<std::uint8_t>(keyBlock[i] ^ 0x5cu);
    }

    std::uint8_t inner[Sha256::kDigestBytes];
    {
        Sha256 ctx;
        ctx.update(ipad, Sha256::kBlockBytes);
        ctx.update(msg, msgLen);
        ctx.final(inner);
    }
    {
        Sha256 ctx;
        ctx.update(opad, Sha256::kBlockBytes);
        ctx.update(inner, Sha256::kDigestBytes);
        ctx.final(out);
    }
}

bool constTimeEqual(const std::uint8_t* a, const std::uint8_t* b, std::size_t n) {
    std::uint8_t diff = 0;
    for (std::size_t i = 0; i < n; ++i) {
        diff = static_cast<std::uint8_t>(diff | (a[i] ^ b[i]));
    }
    return diff == 0;
}

}  // namespace UplinkAuth
