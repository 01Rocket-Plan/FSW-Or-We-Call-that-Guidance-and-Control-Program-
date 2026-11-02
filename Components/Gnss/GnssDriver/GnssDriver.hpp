// ======================================================================
// \title  GnssDriver.hpp
// \author GNSS組（梁承業）
// \brief  TEL0157 真驅動 F´ 外殼；解析／失鎖表在 GnssCore
//
// 為何拆成「外殼＋Core」：
//   Core 不依賴 F´，可在主機單測（T1–T12）驗證 Edward 失鎖表、分頻、emit／seq。
//   本檔只做：prmDb→GnssCoreParams、uartRecv→ring、schedIn→tick、sink→tlm／event／fixOut。
//
// 安裝：DS／US 皆可。Base ID 0x5000。schedIn＝rg100Hz（10 ms）。
// Gate C6：不自己中止；E4：完全逾時後報錯＝FIX_FAILED＋環境訊息一次。
// ======================================================================
#ifndef COMPONENTS_GNSSDRIVER_HPP
#define COMPONENTS_GNSSDRIVER_HPP

#include "Components/Gnss/GnssDriver/GnssDriverComponentAc.hpp"
#include "Components/Gnss/GnssDriver/GnssCore.hpp"
#include <cstdint>

namespace Components {

// UART 位元組環形緩衝：uartRecv 非同步推入，Core 僅在 poll 時刻 readBytes 一次。
// CAP=2048：與 GnssCore::kPollReadMax 同量級——SLOW 2 s @9600 堆積含 GSV ≈1920 B，
// 一次 poll 讀完避免落後；滿則丟最舊以外的新資料（push 時 break），不阻塞匯流排回呼。
class GnssUartAdapter final : public Gnss::IGnssUart {
 public:
  static constexpr U32 CAP = 2048;
  int32_t readBytes(uint8_t* buf, uint32_t max) override;
  void push(const uint8_t* data, U32 len);
  void setBusError() { m_busError = true; }  // 下一 read 回 -1 → Core 進 FAULT
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
  // rg100Hz：每拍讀 prm、必要時 start、再 tick；不 sleep。
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  // ByteStream → ring；OTHER_ERROR → 標 BUS，供下一次 poll 觸發 FAULT。
  void uartRecv_handler(FwIndexType portNum, Fw::Buffer& buffer,
                        const Drv::ByteStreamStatus& status) override;

  // IGnssSink：Core 回呼 → F´ 遙測／事件／fixOut（只接 tlmSend／sdLogger，C6 閉環不依賴）。
  void onFix(const Gnss::GnssFix& fix) override;
  void onStateChange(Gnss::DrvState from, Gnss::DrvState to) override;
  void onLockStageChange(Gnss::LockStage from, Gnss::LockStage to) override;
  void onFault(Gnss::GnssFault fault, uint32_t count) override;
  void onFixFailedOpenSky() override;

  Gnss::GnssCoreParams readParams();  // prmDb → Core；rgPeriod_ms 固定 10（rg100Hz）
  static Components::GnssLockStage toFppLock(Gnss::LockStage s);

  GnssUartAdapter m_uart;
  Gnss::GnssCore m_core;
  bool m_started;
  U32 m_faultCount;
  F32 m_lastDataAge;
};

}  // namespace Components
#endif
