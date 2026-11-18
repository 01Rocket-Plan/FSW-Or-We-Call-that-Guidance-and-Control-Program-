// LoraTimeOnAir：公式出處見 LoraTimeOnAir.hpp（Semtech SX1276/77/78/79 datasheet Rev. 4 §4.1.1.7）。
#include "Components/Wireless/RfDriverSim/LoraTimeOnAir.hpp"

namespace Components {

namespace {
// datasheet §6.4 RegModemConfig1.Bw 可選值（Hz，照表上寫的 kHz 值）
constexpr uint32_t kBwTable_Hz[] = {7800u,  10400u, 15600u,  20800u,  31250u,
                                    41700u, 62500u, 125000u, 250000u, 500000u};
}  // namespace

bool loraConfigValid(const LoraModemConfig& c) {
  if (c.sf < 6u || c.sf > 12u) return false;
  if (c.cr < 1u || c.cr > 4u) return false;
  if (c.sf == 6u && c.explicitHeader) return false;  // §4.1.1.2：SF6 只能隱式標頭
  bool bwOk = false;
  for (uint32_t bw : kBwTable_Hz) {
    if (bw == c.bw_Hz) bwOk = true;
  }
  return bwOk;
}

double loraSymbolTime_us(const LoraModemConfig& c) {
  if (!loraConfigValid(c)) return 0.0;
  // Tsym = 2^SF / BW（§4.1.1.5）
  return static_cast<double>(1u << c.sf) * 1.0e6 / static_cast<double>(c.bw_Hz);
}

uint32_t loraLowDataRateOptimize(const LoraModemConfig& c) {
  switch (c.ldro) {
    case LoraLdro::ON:
      return 1u;
    case LoraLdro::OFF:
      return 0u;
    case LoraLdro::AUTO:
    default:
      return (loraSymbolTime_us(c) > kLoraLdroMandatedSymbol_us) ? 1u : 0u;
  }
}

uint32_t loraPayloadSymbols(const LoraModemConfig& c, uint32_t payloadLen) {
  if (!loraConfigValid(c)) return 0u;
  if (payloadLen < 1u || payloadLen > kLoraMaxPayload) return 0u;
  const int32_t sf = static_cast<int32_t>(c.sf);
  const int32_t crc = c.crcOn ? 1 : 0;
  const int32_t ih = c.explicitHeader ? 0 : 1;
  const int32_t de = static_cast<int32_t>(loraLowDataRateOptimize(c));
  const int32_t num = 8 * static_cast<int32_t>(payloadLen) - 4 * sf + 28 + 16 * crc - 20 * ih;
  const int32_t den = 4 * (sf - 2 * de);  // SF≥6、DE≤1 → den ≥ 16 > 0
  int32_t blocks = 0;
  if (num > 0) {
    blocks = (num + den - 1) / den;  // ceil，num>0 時成立
  }
  // max(…, 0)：num ≤ 0 時 blocks 維持 0
  return 8u + static_cast<uint32_t>(blocks) * (static_cast<uint32_t>(c.cr) + 4u);
}

double loraTimeOnAir_us(const LoraModemConfig& c, uint32_t payloadLen) {
  const uint32_t nPayload = loraPayloadSymbols(c, payloadLen);
  if (nPayload == 0u) return 0.0;
  const double tSym = loraSymbolTime_us(c);
  const double tPreamble = (static_cast<double>(c.preambleLen) + 4.25) * tSym;
  return tPreamble + static_cast<double>(nPayload) * tSym;
}

}  // namespace Components
