// FramerCore：§2 空中幀組幀（大端；CRC-32 涵蓋長度..酬載）
// 被動、不 sleep、不讀系統時間；時間欄只轉發呼叫端 t_s。
#ifndef WIRELESS_FRAMERCORE_HPP
#define WIRELESS_FRAMERCORE_HPP

#include <cstdint>
#include <cstddef>

namespace Components {

enum class FrameType : uint8_t {
  TLM = 0x01,
  EVT = 0x02,
  FILE = 0x03,
  CMD = 0x04,
  ACK = 0x05,
};

enum class FrameSource : uint8_t {
  DS = 0x01,
  US = 0x02,
  GROUND = 0x10,
};

struct FrameRequest {
  FrameType type = FrameType::TLM;
  FrameSource source = FrameSource::DS;
  uint16_t apid = 0;
  uint32_t seq = 0;
  uint8_t bootCount = 0;
  double t_s = 0.0;
  bool valid = false;
  const uint8_t* payload = nullptr;
  uint16_t payloadLen = 0;
};

struct FrameResult {
  const uint8_t* data = nullptr;
  size_t len = 0;
  bool ok = false;
};

enum class FramerError : uint8_t {
  NONE = 0,
  PAYLOAD_TOO_LARGE = 1,
  BUFFER_TOO_SMALL = 2,
  BAD_APID = 3,
  NULL_PAYLOAD = 4,
};

class IFramerEvents {
 public:
  virtual ~IFramerEvents() = default;
  virtual void onFramerError(FramerError code, uint32_t detail) = 0;
};

static constexpr uint32_t FRAMER_SYNC = 0xDEADBEEFu;
static constexpr size_t FRAMER_OFF_SYNC = 0;
static constexpr size_t FRAMER_OFF_LEN = 4;
static constexpr size_t FRAMER_OFF_VER_TYPE = 8;
static constexpr size_t FRAMER_OFF_SOURCE = 9;
static constexpr size_t FRAMER_OFF_APID = 10;
static constexpr size_t FRAMER_OFF_SEQ = 12;
static constexpr size_t FRAMER_OFF_BOOT = 16;
static constexpr size_t FRAMER_OFF_TS = 17;
static constexpr size_t FRAMER_OFF_VALID = 25;
static constexpr size_t FRAMER_OFF_PAYLOAD = 26;
static constexpr size_t FRAMER_TRAILER_CRC = 4;
static constexpr size_t FRAMER_HEADER_BYTES = 26;

class FramerCore {
 public:
  FramerCore() = default;
  void setFrameMax(size_t frameMax_B);
  size_t frameMax() const { return m_frameMax; }
  void setEventSink(IFramerEvents* sink) { m_events = sink; }
  FrameResult build(const FrameRequest& req, uint8_t* out, size_t outCap);
  static uint32_t crc32(const uint8_t* data, size_t len);

 private:
  size_t m_frameMax = 256;  // LoRa-02 256 Byte buffer
  IFramerEvents* m_events = nullptr;
  void emitError(FramerError e, uint32_t detail);
};

}  // namespace Components
#endif
