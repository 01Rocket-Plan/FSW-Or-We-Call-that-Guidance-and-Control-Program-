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
//   - §4.1.1.6 "Low Data Rate Optimization"（p.31）LowDataRateOptimize "mandated when the symbol
//     duration exceeds 16ms"。
//   - §6.4 暫存器表 RegModemConfig1.Bw：7.8/10.4/15.6/20.8/31.25/41.7/62.5/125/250/500 kHz。
//
// 整數算法（issue #60 第 2 項：不用浮點、沒有 UB）：
//   SX1278 的頻寬全部是 500 kHz / d，d ∈ {64,48,32,24,16,12,8,4,2,1}
//   （7.8 kHz 實際是 7812.5 Hz、10.4 kHz 是 10416.67 Hz…，Semtech LoRa Calculator 也用這些精確值）。
//   所以 Tsym = 2^SF · d / 500 kHz = 2^(SF+1) · d µs，一定是整數 µs；
//   Tpacket = (4·npreamble + 17 + 4·npayload) · 2^(SF−1) · d µs（SF ≥ 6，也是整數），用 uint64 算不會溢位。
//
// 純函式：不配置記憶體、不讀系統時間、不 sleep。
// 預設值見 LoraModemConfig（SF7／125 kHz／4:5 已定案）。頻段仍在評估（433 MHz 法規問題），定案後可能換頻段／硬體。
#ifndef WIRELESS_LORATIMEONAIR_HPP
#define WIRELESS_LORATIMEONAIR_HPP

#include <cstdint>

namespace Components {

//! LowDataRateOptimize 設定
enum class LoraLdro : uint8_t {
  OFF = 0,
  ON = 1,
  AUTO = 2,  //!< 依 datasheet §4.1.1.6：Tsym > 16 ms 時開啟，否則關閉（SF11／SF12 @125 kHz 會開）
};

//! RegModemConfig1.Bw 編碼（datasheet §6.4），數值＝暫存器欄位值
enum class LoraBw : uint8_t {
  BW_7_8K = 0,    //!< 7.8125 kHz
  BW_10_4K = 1,   //!< 10.4167 kHz
  BW_15_6K = 2,   //!< 15.625 kHz
  BW_20_8K = 3,   //!< 20.8333 kHz
  BW_31_25K = 4,  //!< 31.25 kHz
  BW_41_7K = 5,   //!< 41.6667 kHz
  BW_62_5K = 6,   //!< 62.5 kHz
  BW_125K = 7,    //!< 125 kHz
  BW_250K = 8,    //!< 250 kHz
  BW_500K = 9,    //!< 500 kHz
};

//! RegModemConfig1.CodingRate 編碼（1..4 ＝ 4/5..4/8）
enum class LoraCr : uint8_t {
  CR_4_5 = 1,
  CR_4_6 = 2,
  CR_4_7 = 3,
  CR_4_8 = 4,
};

//! LoRa 數據機設定（空中時間公式的全部輸入）
//! 預設：SF7／125 kHz／4:5（Edward 定案）；前導 8／顯式標頭／CRC 開 待 Edward 定；LDRO 自動
struct LoraModemConfig {
  uint8_t sf = 7;                  //!< SpreadingFactor 6..12（定案 SF7）
  uint32_t bw_Hz = 125000;         //!< 標稱頻寬 Hz，只收 §6.4 表上的值（定案 125 kHz）
  uint8_t cr = 1;                  //!< CodingRate 1..4 ＝ 4/5..4/8（定案 4/5）
  uint16_t preambleLen = 8;        //!< PreambleLength（待 Edward 定）
  bool explicitHeader = true;      //!< 顯式標頭（待 Edward 定；SF6 必須 false）
  bool crcOn = true;               //!< 酬載 CRC（待 Edward 定）
  LoraLdro ldro = LoraLdro::AUTO;  //!< LowDataRateOptimize（自動＝Tsym > 16 ms 開）
};

//! datasheet §4.1.1.6：符號長度超過 16 ms 必須開 LowDataRateOptimize
static constexpr uint32_t kLoraLdroMandatedSymbolUs = 16000u;
static constexpr double kLoraLdroMandatedSymbol_us = 16000.0;  //!< 舊名（double），同上
//! datasheet §4.1.1.7：PL 範圍 1..255（同 RegPayloadLength 8 bit）
static constexpr uint32_t kLoraMaxPayload = 255;

//! LoraBw → 標稱頻寬 Hz（§6.4 表上寫的值）；無效編碼回 0
uint32_t loraBwHz(LoraBw bw);

//! 標稱頻寬 Hz → LoraBw；不在 §6.4 表內回 false（out 不動）
bool loraBwFromHz(uint32_t bw_Hz, LoraBw& out);

//! 500 kHz / BW 的整數除數 d（見檔頭）；bw_Hz 不在 §6.4 表內回 0
uint32_t loraBwDivisor(uint32_t bw_Hz);

//! 設定是否為 datasheet 允許的組合（SF、BW、CR 範圍；SF6 必須隱式標頭）
bool loraConfigValid(const LoraModemConfig& c);

//! 符號長度 Tsym（整數 µs，精確值）。設定無效回 0。
uint32_t loraSymbolTimeUs(const LoraModemConfig& c);

//! 實際使用的 DE（0／1）：依 ldro 設定（AUTO 依 16 ms 規則）
uint32_t loraLowDataRateOptimize(const LoraModemConfig& c);

//! 酬載符號數 npayload（datasheet §4.1.1.7）。設定或 PL 無效回 0。
uint32_t loraPayloadSymbols(const LoraModemConfig& c, uint32_t payloadLen);

//! 整包空中時間 Tpacket（整數 µs，精確值）。設定或 PL 無效回 0。
uint64_t loraTimeOnAirUs(const LoraModemConfig& c, uint32_t payloadLen);

//! 空中時間要佔幾個 tick：ceil(Tpacket / tickPeriodUs)。設定／PL 無效或 tickPeriodUs==0 回 0。
uint32_t loraAirtimeTicks(const LoraModemConfig& c, uint32_t payloadLen, uint32_t tickPeriodUs);

//! 相容舊 API（double）：值＝上面整數版本
double loraSymbolTime_us(const LoraModemConfig& c);
double loraTimeOnAir_us(const LoraModemConfig& c, uint32_t payloadLen);

}  // namespace Components
#endif
