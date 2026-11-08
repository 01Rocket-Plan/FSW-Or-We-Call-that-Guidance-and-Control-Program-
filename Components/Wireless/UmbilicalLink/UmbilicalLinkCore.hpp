// UmbilicalLinkCore：箭體 ↔ 發射架 臍帶鏈路（DsMcu）— Issue #24
//
// 有線臍帶 UART，雙向：
//   箭 → 架：每 period_s 送一幀 RocketStatus（飛行模式、BIT 旗標、ready、電池電壓）
//   架 → 箭：收 PadToRocket（架狀態、倒數、點火／推力確認／夾持已鬆旗標）
//
// 方案 A（Edward 2026-11-08，Issue #24）：鬆夾持由架決定，箭用 ready 否決。
// 架 → 箭 只是狀態回報，本元件不依它做任何決定、也不回「放行」；
// 要不要用架狀態（例如倒數中預先準備起飛偵測）是 FlightMode 的事（CONTRIBUTING 7）。
// 點火指令仍只走「地面 → 架」，火箭不在點火迴路裡（gate C15 第三輪）。
//
// ready＝硬擋 BIT 全過（E2：IMU、電壓、舵回中、氣壓）且 BIT 資料夠新
//       且模式仍是 PAD_IDLE／ARMED。警告項（SD、遙測）只回報、不擋 ready。
//
// 起飛後臍帶拔脫是預期行為（SSS「箭體、發射架通訊」）：
// 寫入失敗、收不到架狀態都只計數／標 padLinkOk=false，不發 warning、不進錯誤狀態。
#ifndef WIRELESS_UMBILICALLINKCORE_HPP
#define WIRELESS_UMBILICALLINKCORE_HPP

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

struct UmbilicalLinkParams {
  double period_s = 0.1;          // 10 Hz
  double bitMaxAge_s = 0.5;       // BIT 資料超過這個年齡 → ready=false
  double padLinkTimeout_s = 0.5;  // 多久沒收到架狀態 → padLinkOk=false
};

class UmbilicalLinkCore {
 public:
  explicit UmbilicalLinkCore(IUmbilicalUart& uart, std::uint8_t bootCount = 0)
      : m_uart(uart), m_bootCount(bootCount) {}

  void configure(const UmbilicalLinkParams& p) { m_p = p; }
  void setMode(std::uint8_t flightMode) { m_mode = flightMode; }
  void setBit(std::uint16_t bitFlags, double now_s) {
    m_bit = bitFlags;
    m_bitAt_s = now_s;
    m_haveBit = true;
  }
  void setBattV(double v) { m_battV = v; }

  //! 每拍呼叫；到期就送一幀 RocketStatus。回傳本拍是否有嘗試送出。
  bool tick(double now_s);

  //! 臍帶 UART 收到的位元組（可分段餵）。只收 PadToRocket，其他型別丟掉。
  void onUmbilicalBytes(const std::uint8_t* data, std::size_t len, double now_s);

  //! 目前會回報的 ready（給測試與遙測）。
  bool computeReady(double now_s) const;

  //! 架狀態是否夠新（臍帶還插著、架還活著）。
  bool padLinkOk(double now_s) const {
    return m_havePad && (now_s - m_lastPadRx_s) <= m_p.padLinkTimeout_s;
  }
  //! 最近一次收到的架狀態（padLinkOk 為 false 時只當歷史值看）。
  const PadToRocketMsg& lastPadStatus() const { return m_pad; }

  std::uint32_t sentFrames() const { return m_sent; }
  std::uint32_t writeFailures() const { return m_writeFail; }
  std::uint32_t rxFrames() const { return m_rx; }
  std::uint32_t rxCrcErrors() const { return m_deframer.crcErrors(); }

 private:
  IUmbilicalUart& m_uart;
  UmbilicalLinkParams m_p{};
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

  StreamDeframer m_deframer;
  PadToRocketMsg m_pad{};
  bool m_havePad = false;
  double m_lastPadRx_s = 0.0;
  std::uint32_t m_rx = 0;
};

}  // namespace PadLink

#endif
