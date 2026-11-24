// LoraTimeOnAir：公式出處見 LoraTimeOnAir.hpp（Semtech SX1276/77/78/79 datasheet Rev. 4 §4.1.1.7）。
// 全部整數運算；所有中間值的上下界寫在註解裡。
#include "Components/Wireless/RfDriver/LoraTimeOnAir.hpp"

namespace Components {

namespace {
struct BwEntry {
  uint32_t nominal_Hz;  // §6.4 表上寫的值
  uint32_t divisor;     // 500 kHz / 精確頻寬
};
// 索引＝RegModemConfig1.Bw 編碼（LoraBw）
constexpr BwEntry kBwTable[] = {
    {7800u, 64u},  {10400u, 48u}, {15600u, 32u},  {20800u, 24u},  {31250u, 16u},
    {41700u, 12u}, {62500u, 8u},  {125000u, 4u},  {250000u, 2u},  {500000u, 1u},
};
constexpr uint32_t kBwCount = sizeof(kBwTable) / sizeof(kBwTable[0]);
}  // namespace

uint32_t loraBwHz(LoraBw bw) {
  const uint32_t i = static_cast<uint32_t>(bw);
  return (i < kBwCount) ? kBwTable[i].nominal_Hz : 0u;
}

bool loraBwFromHz(uint32_t bw_Hz, LoraBw& out) {
  for (uint32_t i = 0; i < kBwCount; ++i) {
    if (kBwTable[i].nominal_Hz == bw_Hz) {
      out = static_cast<LoraBw>(i);
      return true;
    }
  }
  return false;
}

uint32_t loraBwDivisor(uint32_t bw_Hz) {
  for (uint32_t i = 0; i < kBwCount; ++i) {
    if (kBwTable[i].nominal_Hz == bw_Hz) return kBwTable[i].divisor;
  }
  return 0u;
}

bool loraConfigValid(const LoraModemConfig& c) {
  if (c.sf < 6u || c.sf > 12u) return false;
  if (c.cr < 1u || c.cr > 4u) return false;
  if (c.sf == 6u && c.explicitHeader) return false;  // §4.1.1.2：SF6 只能隱式標頭
  if (c.ldro != LoraLdro::OFF && c.ldro != LoraLdro::ON && c.ldro != LoraLdro::AUTO) return false;
  return loraBwDivisor(c.bw_Hz) != 0u;
}

uint32_t loraSymbolTimeUs(const LoraModemConfig& c) {
  if (!loraConfigValid(c)) return 0u;
  // Tsym = 2^SF / (500 kHz / d) = 2^(SF+1)·d µs；SF ≤ 12、d ≤ 64 → ≤ 524 288
  return (1u << (static_cast<uint32_t>(c.sf) + 1u)) * loraBwDivisor(c.bw_Hz);
}

uint32_t loraLowDataRateOptimize(const LoraModemConfig& c) {
  switch (c.ldro) {
    case LoraLdro::ON:
      return 1u;
    case LoraLdro::OFF:
      return 0u;
    case LoraLdro::AUTO:
    default:
      return (loraSymbolTimeUs(c) > kLoraLdroMandatedSymbolUs) ? 1u : 0u;
  }
}

uint32_t loraPayloadSymbols(const LoraModemConfig& c, uint32_t payloadLen) {
  if (!loraConfigValid(c)) return 0u;
  if (payloadLen < 1u || payloadLen > kLoraMaxPayload) return 0u;
  const int32_t sf = static_cast<int32_t>(c.sf);
  const int32_t crc = c.crcOn ? 1 : 0;
  const int32_t ih = c.explicitHeader ? 0 : 1;
  const int32_t de = static_cast<int32_t>(loraLowDataRateOptimize(c));
  // PL ≤ 255 → num ∈ [8−48+28−20, 2040−24+28+16] ＝ [−32, 2060]，int32 不會溢位
  const int32_t num = 8 * static_cast<int32_t>(payloadLen) - 4 * sf + 28 + 16 * crc - 20 * ih;
  const int32_t den = 4 * (sf - 2 * de);  // SF ≥ 6、DE ≤ 1 → den ≥ 16 > 0
  int32_t blocks = 0;
  if (num > 0) {
    blocks = (num + den - 1) / den;  // ceil（num > 0、den > 0）
  }
  // max(…, 0)：num ≤ 0 時 blocks 維持 0；blocks ≤ 129 → 結果 ≤ 8 + 129·8
  return 8u + static_cast<uint32_t>(blocks) * (static_cast<uint32_t>(c.cr) + 4u);
}

uint64_t loraTimeOnAirUs(const LoraModemConfig& c, uint32_t payloadLen) {
  const uint32_t nPayload = loraPayloadSymbols(c, payloadLen);
  if (nPayload == 0u) return 0u;
  // Tpacket = (npreamble + 4.25 + npayload)·Tsym = (4·npreamble + 17 + 4·npayload)·Tsym/4
  // Tsym/4 = 2^(SF−1)·d（SF ≥ 6 → 整數）
  // 上界：(4·65535 + 17 + 4·1040)·2^11·64 ≈ 3.5e10 < 2^64
  const uint64_t quarters = 4u * static_cast<uint64_t>(c.preambleLen) + 17u + 4u * static_cast<uint64_t>(nPayload);
  const uint64_t quarterSym_us =
      (static_cast<uint64_t>(1u) << (static_cast<uint32_t>(c.sf) - 1u)) * loraBwDivisor(c.bw_Hz);
  return quarters * quarterSym_us;
}

uint32_t loraAirtimeTicks(const LoraModemConfig& c, uint32_t payloadLen, uint32_t tickPeriodUs) {
  if (tickPeriodUs == 0u) return 0u;
  const uint64_t toa = loraTimeOnAirUs(c, payloadLen);
  if (toa == 0u) return 0u;
  const uint64_t ticks = (toa + tickPeriodUs - 1u) / tickPeriodUs;  // ceil；toa < 2^36，不會溢位
  return (ticks > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(ticks);
}

double loraSymbolTime_us(const LoraModemConfig& c) { return static_cast<double>(loraSymbolTimeUs(c)); }

double loraTimeOnAir_us(const LoraModemConfig& c, uint32_t payloadLen) {
  return static_cast<double>(loraTimeOnAirUs(c, payloadLen));
}

}  // namespace Components
