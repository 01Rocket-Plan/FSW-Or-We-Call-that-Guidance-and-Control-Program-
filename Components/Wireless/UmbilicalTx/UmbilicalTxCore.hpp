// UmbilicalTxCore：箭體 → 發射架 臍帶狀態（DsMcu）— Issue #24
//
// 有線臍帶 UART，單向（箭 → 架）。箭上不從臍帶收任何東西：
// 點火指令只走「地面 → 架」，火箭不在點火迴路裡（gate C15 第三輪）。
//
// 每 period_s 送一幀 RocketStatus：飛行模式、BIT 旗標、ready、電池電壓。
// ready＝硬擋 BIT 全過（E2：IMU、電壓、舵回中、氣壓）且 BIT 資料夠新
//       且模式仍是 PAD_IDLE／ARMED。警告項（SD、遙測）只回報、不擋 ready。
//
// 起飛後臍帶拔脫是預期行為（SSS「箭體、發射架通訊」）：
// 寫入失敗只計數，不發 warning、不進任何錯誤狀態，也不影響飛行。
#ifndef WIRELESS_UMBILICALTXCORE_HPP
#define WIRELESS_UMBILICALTXCORE_HPP

#include "PadLinkCodec.hpp"

#include <cstddef>
#include <cstdint>

namespace PadLink {

//! RocketTypes.FlightModeId 的數值（只用到架上兩個）。
static constexpr std::uint8_t kModePadIdle = 0;
static constexpr std::uint8_t kModeArmed = 1;

class IUmbilicalUart {
 public:
  virtual ~IUmbilicalUart() = default;
  //! 回傳實際寫出的位元組數；臍帶斷了可以回 0 或負值。
  virtual int write(const std::uint8_t* data, std::size_t len) = 0;
};

struct UmbilicalTxParams {
  double period_s = 0.1;     // 10 Hz
  double bitMaxAge_s = 0.5;  // BIT 資料超過這個年齡 → ready=false
};

class UmbilicalTxCore {
 public:
  explicit UmbilicalTxCore(IUmbilicalUart& uart, std::uint8_t bootCount = 0)
      : m_uart(uart), m_bootCount(bootCount) {}

  void configure(const UmbilicalTxParams& p) { m_p = p; }
  void setMode(std::uint8_t flightMode) { m_mode = flightMode; }
  void setBit(std::uint16_t bitFlags, double now_s) {
    m_bit = bitFlags;
    m_bitAt_s = now_s;
    m_haveBit = true;
  }
  void setBattV(double v) { m_battV = v; }

  //! 每拍呼叫；到期就送一幀。回傳本拍是否有嘗試送出。
  bool tick(double now_s);

  //! 目前會回報的 ready（給測試與遙測）。
  bool computeReady(double now_s) const;

  std::uint32_t sentFrames() const { return m_sent; }
  std::uint32_t writeFailures() const { return m_writeFail; }

 private:
  IUmbilicalUart& m_uart;
  UmbilicalTxParams m_p{};
  std::uint8_t m_bootCount;
  std::uint8_t m_mode = kModePadIdle;
  std::uint16_t m_bit = 0;
  double m_bitAt_s = 0.0;
  bool m_haveBit = false;
  double m_battV = 0.0;
  bool m_haveTx = false;
  double m_lastTx_s = 0.0;
  std::uint16_t m_seq = 0;
  std::uint32_t m_sent = 0;
  std::uint32_t m_writeFail = 0;
};

}  // namespace PadLink

#endif
