// ======================================================================
// \title  PlatformCore.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  平台核心：主迴路看門狗踢狗、迴圈間隔監測、重開機原因。
//         純 C++14；硬體差異全在 IPlatformHal 後面。
//
//   SIL（Linux）：LinuxPlatformHal（本 PR）——踢狗只計數，重開機原因固定 POWER_ON
//   ESP32       ：Esp32PlatformHal（下一個 PR）——esp_task_wdt_reset()、esp_reset_reason()
//
// 看門狗咬的是「主迴路卡死」（gates：I2C hang），不是「再算一次」：
// Platform 排在 rg1kHz 的最後一個，鏈上任何一個元件卡住，這裡就不會被叫到、不會踢狗。
// ======================================================================
#ifndef COMPONENTS_MCU_PLATFORM_PLATFORMCORE_HPP
#define COMPONENTS_MCU_PLATFORM_PLATFORMCORE_HPP

#include <cstdint>

namespace Mcu {

//! 重開機原因（數值＝Platform.fpp ResetReason）
enum class ResetCause : uint8_t {
  UNKNOWN = 0,
  POWER_ON = 1,
  SOFTWARE = 2,
  WATCHDOG = 3,   //!< 主迴路卡死被咬
  BROWNOUT = 4,   //!< 欠壓重開
  PANIC = 5,      //!< 例外／assert
};

class IPlatformHal {
 public:
  virtual ~IPlatformHal() = default;
  virtual void kickWatchdog() = 0;
  virtual ResetCause resetReason() const = 0;
};

//! SIL 用：不碰任何硬體。
class LinuxPlatformHal final : public IPlatformHal {
 public:
  void kickWatchdog() override { ++m_kicks; }
  ResetCause resetReason() const override { return ResetCause::POWER_ON; }
  uint32_t kicks() const { return m_kicks; }

 private:
  uint32_t m_kicks = 0;
};

struct PlatformParams {
  float loopPeriod_s = 0.001f;   //!< 1 kHz
  float overrun_s = 0.0015f;     //!< 兩拍間隔超過這個算一次超時
};

struct PlatformStats {
  uint32_t ticks = 0;
  uint32_t overruns = 0;
  float lastDt_s = 0.0f;
  float maxDt_s = 0.0f;
};

class PlatformCore {
 public:
  explicit PlatformCore(IPlatformHal& hal) : m_hal(&hal) {}
  void setHal(IPlatformHal& hal) { m_hal = &hal; }
  void setParams(const PlatformParams& p) { m_p = p; }

  //! rg1kHz 最後一個成員每拍叫一次。回傳 true＝這拍超時。
  bool tick(double now_s);

  ResetCause resetReason() const { return m_hal->resetReason(); }
  //! 上次重開是不是「不正常」（看門狗／欠壓／例外）→ FlightMode 應進安全態（FM-20）
  bool abnormalReset() const;

  const PlatformStats& stats() const { return m_st; }
  void clearMax() { m_st.maxDt_s = 0.0f; }

 private:
  IPlatformHal* m_hal;
  PlatformParams m_p;
  PlatformStats m_st;
  bool m_have = false;
  double m_last_s = 0.0;
};

}  // namespace Mcu

#endif
