// GnssIfaces.hpp - GnssCore 依賴的介面 (與 F´ 無關，可主機單測)
#pragma once
#include <cstdint>
#include "GnssTypes.hpp"
namespace Gnss {
// UART: 非阻塞，回傳實際讀到的位元組數 (0 = 無資料, <0 = 匯流排錯誤)。每個 tick 最多呼叫一次。
class IGnssUart {
 public:
  virtual ~IGnssUart() = default;
  virtual int32_t readBytes(uint8_t* buf, uint32_t max) = 0;
};
enum class GnssFault : uint8_t { NONE = 0, NO_DATA, BAD_CHECKSUM, TRUNCATED, STUCK, BUS_ERROR, SELFTEST_FAIL };
// 輸出回呼: 時間由核心呼叫端注入 (F´ 時間服務)，核心不讀系統時間。
class IGnssSink {
 public:
  virtual ~IGnssSink() = default;
  virtual void onFix(const GnssFix& fix) = 0;
  virtual void onStateChange(DrvState from, DrvState to) = 0;
  virtual void onFault(GnssFault fault, uint32_t count) = 0;
};
}  // namespace Gnss
