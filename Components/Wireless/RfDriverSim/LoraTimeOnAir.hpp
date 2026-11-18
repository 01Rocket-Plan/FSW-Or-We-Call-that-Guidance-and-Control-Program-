// LoraTimeOnAir：Semtech SX1276/77/78/79 LoRa 封包空中時間（time-on-air）公式。
//
// 出處（每個數字都來自這份文件）：
//   Semtech, "SX1276/77/78/79 — 137 MHz to 1020 MHz Low Power Long Range Transceiver",
//   Datasheet Rev. 4, March 2015。
//   - §4.1.1.5（p.28）符號速率 Rs = BW / 2^SF  →  Tsym = 2^SF / BW
//   - §4.1.1.7（p.31）Time on air：
//       Tpreamble = (npreamble + 4.25) · Tsym
//       npayload  = 8 + max( ceil( (8PL − 4SF + 28 + 16CRC − 20IH) / (4(SF − 2DE)) ) · (CR + 4), 0 )
//       Tpacket   = Tpreamble + npayload · Tsym
//     PL = 1..255，SF = 6..12，IH = 1 隱式標頭／0 顯式標頭，DE = 1 開 LowDataRateOptimize，
//     CR = 1..4（4/5..4/8）。
//   - §4.1.1.2（p.27）SF6 只能用隱式標頭（"The header must be set to Implicit mode"）。
//   - §4.1.1.6 "Low Data Rate Optimization"（p.31）LowDataRateOptimize "mandated when the symbol duration exceeds 16ms"。
//   - §6.4 暫存器表（p.112–114）reset 值：Bw=0111（125 kHz）、CodingRate=001（4/5）、
//     ImplicitHeaderModeOn=0（顯式標頭）、SpreadingFactor=7、RxPayloadCrcOn=0、
//     PreambleLength=8、LowDataRateOptimize=0；BW 只能是 7.8/10.4/15.6/20.8/31.25/41.7/62.5/
//     125/250/500 kHz。
//
// 預設值＝上面 SX1278 的暫存器 reset 值（repo 目前沒有任何 SF／BW／CR 設定，見 issue #60）。
// 無線組可再改。
//
// 純函式：不配置記憶體、不讀系統時間、不 sleep。
#ifndef WIRELESS_LORATIMEONAIR_HPP
#define WIRELESS_LORATIMEONAIR_HPP

#include <cstdint>

namespace Components {

//! LowDataRateOptimize 設定
enum class LoraLdro : uint8_t {
  OFF = 0,
  ON = 1,
  AUTO = 2,  //!< 依 datasheet §4.1.1.6：Tsym > 16 ms 時開啟，否則關閉
};

//! LoRa 數據機設定（預設＝SX1278 暫存器 reset 值，datasheet §6.4；無線組可再改）
struct LoraModemConfig {
  uint8_t sf = 7;                 //!< SpreadingFactor 6..12（reset 0x07）
  uint32_t bw_Hz = 125000;        //!< 訊號頻寬（reset Bw=0111 → 125 kHz）
  uint8_t cr = 1;                 //!< CodingRate 1..4 ＝ 4/5..4/8（reset 001 → 4/5）
  uint16_t preambleLen = 8;       //!< PreambleLength（reset 0x0008）
  bool explicitHeader = true;     //!< ImplicitHeaderModeOn=0（reset）→ 顯式標頭
  bool crcOn = false;             //!< RxPayloadCrcOn（reset 0）
  LoraLdro ldro = LoraLdro::AUTO; //!< reset 為 0；AUTO 在預設 SF7/125 kHz 下也等於關閉
};

//! datasheet §4.1.1.6：符號長度超過 16 ms 必須開 LowDataRateOptimize
static constexpr double kLoraLdroMandatedSymbol_us = 16000.0;
//! datasheet §4.1.1.7：PL 範圍 1..255（同 RegPayloadLength 8 bit）
static constexpr uint32_t kLoraMaxPayload = 255;

//! 設定是否為 datasheet 允許的組合（SF、BW、CR 範圍；SF6 必須隱式標頭）
bool loraConfigValid(const LoraModemConfig& c);

//! 符號長度 Tsym（µs）= 2^SF / BW。設定無效回 0。
double loraSymbolTime_us(const LoraModemConfig& c);

//! 實際使用的 DE（0／1）：依 ldro 設定（AUTO 依 16 ms 規則）
uint32_t loraLowDataRateOptimize(const LoraModemConfig& c);

//! 酬載符號數 npayload（datasheet §4.1.1.7）。設定或 PL 無效回 0。
uint32_t loraPayloadSymbols(const LoraModemConfig& c, uint32_t payloadLen);

//! 整包空中時間 Tpacket（µs）。設定或 PL 無效回 0。
double loraTimeOnAir_us(const LoraModemConfig& c, uint32_t payloadLen);

}  // namespace Components
#endif
