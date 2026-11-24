// LogicalPacket：tlmSend → comQueue → framer 之間的「邏輯封包」（板內記憶體格式，不上空中）。
// 空中幀格式見 Framer（§2）；這裡只帶 framer 組幀需要、但 Svc.ComDataWithContext 埠帶不到的標頭欄位。
//
// 佈局（大端，與空中幀同序，方便對照）：
//   [0] type  [1] source  [2..3] apid  [4..7] seq U32  [8] bootCount  [9..16] t_s F64  [17] valid  [18..] payload
//
// 純 inline、不配置記憶體；header-only（不需連結依賴）。
#ifndef WIRELESS_LOGICALPACKET_HPP
#define WIRELESS_LOGICALPACKET_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Components {
namespace WirelessPkt {

static constexpr size_t kHeaderBytes = 18;
//! 空中幀上限 255 B − 幀頭 26 B − CRC 4 B ＝ 225 B 酬載
static constexpr size_t kMaxPayload = 225;
static constexpr size_t kMaxBytes = kHeaderBytes + kMaxPayload;

struct Header {
  uint8_t type = 0;
  uint8_t source = 0;
  uint16_t apid = 0;
  uint32_t seq = 0;
  uint8_t bootCount = 0;
  double t_s = 0.0;
  bool valid = false;
};

//! 寫入 out；回傳總長度，cap 不夠或 payloadLen 過大回 0
inline size_t encode(const Header& h, const uint8_t* payload, size_t payloadLen, uint8_t* out, size_t cap) {
  if (payloadLen > kMaxPayload || (payload == nullptr && payloadLen != 0)) return 0;
  const size_t total = kHeaderBytes + payloadLen;
  if (out == nullptr || cap < total) return 0;
  uint64_t bits = 0;
  static_assert(sizeof(double) == sizeof(uint64_t), "F64 layout");
  std::memcpy(&bits, &h.t_s, sizeof(bits));
  out[0] = h.type;
  out[1] = h.source;
  out[2] = static_cast<uint8_t>(h.apid >> 8);
  out[3] = static_cast<uint8_t>(h.apid & 0xFFu);
  for (int i = 0; i < 4; ++i) out[4 + i] = static_cast<uint8_t>((h.seq >> (24 - 8 * i)) & 0xFFu);
  out[8] = h.bootCount;
  for (int i = 0; i < 8; ++i) out[9 + i] = static_cast<uint8_t>((bits >> (56 - 8 * i)) & 0xFFu);
  out[17] = h.valid ? 1u : 0u;
  if (payloadLen > 0) std::memcpy(out + kHeaderBytes, payload, payloadLen);
  return total;
}

//! 解出標頭；payload 指向 in 內部。長度不足或過長回 false
inline bool decode(const uint8_t* in, size_t len, Header& h, const uint8_t*& payload, size_t& payloadLen) {
  if (in == nullptr || len < kHeaderBytes || len > kMaxBytes) return false;
  h.type = in[0];
  h.source = in[1];
  h.apid = static_cast<uint16_t>((static_cast<uint16_t>(in[2]) << 8) | in[3]);
  h.seq = 0;
  for (int i = 0; i < 4; ++i) h.seq = (h.seq << 8) | in[4 + i];
  h.bootCount = in[8];
  uint64_t bits = 0;
  for (int i = 0; i < 8; ++i) bits = (bits << 8) | in[9 + i];
  std::memcpy(&h.t_s, &bits, sizeof(bits));
  h.valid = in[17] != 0;
  payload = in + kHeaderBytes;
  payloadLen = len - kHeaderBytes;
  return true;
}

}  // namespace WirelessPkt
}  // namespace Components
#endif
