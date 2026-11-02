// ======================================================================
// \title  GnssDriver.hpp
// \author GNSS組（梁承業）
// \brief  TEL0157 真驅動 F´ 外殼；解析／失鎖表在 GnssCore
// 安裝位置：DS／US 皆可。Base ID 0x5000。schedIn＝rg100Hz。
// Edward 2026-11-02 失鎖表；Gate C6／E4 只引 ID。
// ======================================================================
#ifndef COMPONENTS_GNSSDRIVER_HPP
#define COMPONENTS_GNSSDRIVER_HPP

#include "Components/Gnss/GnssDriver/GnssDriverComponentAc.hpp"
#include "Components/Gnss/GnssDriver/GnssCore.hpp"
#include <cstdint>

namespace Components {

class GnssUartAdapter final : public Gnss::IGnssUart {
 public:
  static constexpr U32 CAP = 2048;
  int32_t readBytes(uint8_t* buf, uint32_t max) override;
  void push(const uint8_t* data, U32 len);
  void setBusError() { m_busError = true; }
  U32 size() const { return m_size; }

 private:
  uint8_t m_buf[CAP];
  U32 m_head = 0, m_tail = 0, m_size = 0;
  bool m_busError = false;
};

class GnssDriver final : public GnssDriverComponentBase, private Gnss::IGnssSink {
 public:
  explicit GnssDriver(const char* compName);
  ~GnssDriver() override = default;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void uartRecv_handler(FwIndexType portNum, Fw::Buffer& buffer,
                        const Drv::ByteStreamStatus& status) override;

  void onFix(const Gnss::GnssFix& fix) override;
  void onStateChange(Gnss::DrvState from, Gnss::DrvState to) override;
  void onLockStageChange(Gnss::LockStage from, Gnss::LockStage to) override;
  void onFault(Gnss::GnssFault fault, uint32_t count) override;
  void onFixFailedOpenSky() override;

  Gnss::GnssCoreParams readParams();
  static Components::GnssLockStage toFppLock(Gnss::LockStage s);

  GnssUartAdapter m_uart;
  Gnss::GnssCore m_core;
  bool m_started;
  U32 m_faultCount;
  F32 m_lastDataAge;
};

}  // namespace Components
#endif
