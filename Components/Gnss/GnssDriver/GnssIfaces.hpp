// GnssIfaces.hpp - GnssCore 依賴的介面 (與 F´ 無關，可主機單測)
#pragma once
#include <cstdint>
#include "GnssTypes.hpp"
namespace Gnss {
class IGnssUart {
 public:
  virtual ~IGnssUart() = default;
  virtual int32_t readBytes(uint8_t* buf, uint32_t max) = 0;
};
// BUS_ERROR／SELFTEST_FAIL／INIT_ACQ_TIMEOUT → FAULT。
// FIX_FAILED → 階段事件＋Edward 原文一次；維持 READY、SLOW 輪詢。
// BAD_CHECKSUM／TRUNCATED 只計數。
enum class GnssFault : uint8_t {
  NONE = 0,
  BAD_CHECKSUM = 1,
  TRUNCATED = 2,
  BUS_ERROR = 3,
  SELFTEST_FAIL = 4,
  INIT_ACQ_TIMEOUT = 5,
  FIX_FAILED = 6
};
class IGnssSink {
 public:
  virtual ~IGnssSink() = default;
  virtual void onFix(const GnssFix& fix) = 0;
  virtual void onStateChange(DrvState from, DrvState to) = 0;
  virtual void onLockStageChange(LockStage from, LockStage to) = 0;
  virtual void onFault(GnssFault fault, uint32_t count) = 0;
  // Edward 2026-11-02：進入 FIX_FAILED 時呼叫一次（驅動發固定中文事件）
  virtual void onFixFailedOpenSky() = 0;
};
}  // namespace Gnss
