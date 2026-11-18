// RfDriverSimDevice：位元組層 RF 模擬（掉包／延遲／BER＋LoRa 空中時間）。不得讀取 truth／飛行狀態。
//
// 空中時間（issue #60）：每次 writeFrame 依 LoraTimeOnAir（Semtech SX1276/77/78/79 datasheet
// Rev. 4 §4.1.1.7）算出這一包要佔幾個 tick；佔線期間再呼叫 writeFrame 回 0（無線電忙、該幀不收），
// RfDriverCore 會記成 TX_FAIL／TxDroppedCount。ready() 不受佔線影響（RfDriverCore 在 READY 狀態
// ready()==false 超過 readyDeadline_s 會進 FAULT，所以不能拿 ready() 表示佔線）。
// 「TX 完成改成等空中時間、不要下一 tick 就完成」是 RfDriverCore 的事（issue #60 第 2 項，無線組）。
#ifndef WIRELESS_RFDRIVERSIMDEVICE_HPP
#define WIRELESS_RFDRIVERSIMDEVICE_HPP

#include "../RfDriver/RfDriverCore.hpp"
#include "Components/Wireless/RfDriverSim/LoraTimeOnAir.hpp"
#include <cstdint>

namespace Components {

enum class RfSimFault : uint8_t {
  NONE = 0,
  INIT_FAIL = 1,
  SELFTEST_FAIL = 2,
  NOT_READY = 3,
  WRITE_FAIL = 4,
};

class RfDriverSimDevice : public IRfBus {
 public:
  RfDriverSimDevice();

  void setDropRate(double r) { m_dropRate = r; }
  void setLatencyTicks(uint32_t n) { m_latencyTicks = n; }
  void setBer(double b) { m_ber = b; }
  void setFault(RfSimFault f) { m_fault = f; }
  //! LoRa 數據機設定（SF／BW／CR／前導碼／標頭／CRC／LDRO）；預設＝SX1278 reset 值，無線組可再改
  void setModem(const LoraModemConfig& c) { m_modem = c; }
  const LoraModemConfig& modem() const { return m_modem; }
  //! 一個 advance() 代表多少 µs。預設 10000（rg100Hz）；RfDriver 掛哪個 rate group 還沒定（issue #60 第 4 項）
  void setTickPeriodUs(uint32_t us) { m_tickPeriod_us = (us == 0u) ? 1u : us; }
  uint32_t tickPeriodUs() const { return m_tickPeriod_us; }
  //! 關掉空中時間模型（只給只驗掉包／計數的舊測試用；預設開）
  void setAirtimeEnabled(bool on) { m_airtimeEnabled = on; }
  //! len 位元組一包要佔幾個 tick（ceil(ToA / tickPeriod)）；設定無效或 len 超出 1..255 回 0
  uint32_t airtimeTicks(uint32_t len) const;
  //! 目前是否還在發送上一包
  bool txBusy() const { return m_airtimeEnabled && (m_ticks < m_busyUntilTick); }
  uint32_t lastAirtimeUs() const { return m_lastAirtime_us; }
  uint32_t framesRejectedBusy() const { return m_framesRejectedBusy; }
  uint32_t framesRejectedLength() const { return m_framesRejectedLength; }
  void advance();
  void pushUplink(const uint8_t* data, uint32_t len);

  uint32_t framesWritten() const { return m_framesWritten; }
  uint32_t framesDropped() const { return m_framesDropped; }
  uint32_t framesCorrupted() const { return m_framesCorrupted; }

  bool init() override;
  int32_t writeFrame(const uint8_t* data, uint32_t len) override;
  int32_t readBytes(uint8_t* data, uint32_t maxLen) override;
  bool selftestLoopback() override;
  bool ready() override;

 private:
  bool corruptByte();
  uint32_t rng();

  static constexpr uint32_t kUplinkCap = 1024;

  RfSimFault m_fault = RfSimFault::NONE;
  double m_dropRate = 0.0;
  double m_ber = 0.0;
  uint32_t m_latencyTicks = 0;
  uint32_t m_ticks = 0;
  uint32_t m_rng = 0x12345678u;

  uint8_t m_uplinkBuf[kUplinkCap]{};
  uint32_t m_uplinkCount = 0;
  uint32_t m_uplinkRd = 0;

  uint32_t m_framesWritten = 0;
  uint32_t m_framesDropped = 0;
  uint32_t m_framesCorrupted = 0;

  LoraModemConfig m_modem{};
  uint32_t m_tickPeriod_us = 10000u;
  bool m_airtimeEnabled = true;
  uint32_t m_busyUntilTick = 0;
  uint32_t m_lastAirtime_us = 0;
  uint32_t m_framesRejectedBusy = 0;
  uint32_t m_framesRejectedLength = 0;
};

}  // namespace Components
#endif
