// RfDriverSimDevice：位元組層 RF 模擬（掉包／延遲／BER）。不得讀取 truth／飛行狀態。
#ifndef WIRELESS_RFDRIVERSIMDEVICE_HPP
#define WIRELESS_RFDRIVERSIMDEVICE_HPP

#include "../RfDriver/RfDriverCore.hpp"
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
};

}  // namespace Components
#endif
