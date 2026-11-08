// TlmSendCore：遙測排程（rg100Hz）；每 APID 獨立 U32 seq；t_s 來自 F´ 時間服務。
// 若 t_s 倒退：發 EVT_TIME_NONMONOTONIC，仍照常送出（不夾時間）。
#ifndef WIRELESS_TLMSENDCORE_HPP
#define WIRELESS_TLMSENDCORE_HPP

#include <cstdint>
#include <cstddef>

namespace Components {

enum class TlmPktType : uint8_t {
  TLM = 0x01,
  EVT = 0x02,
  FILE = 0x03,
  CMD = 0x04,
  ACK = 0x05,
};

enum class TlmSource : uint8_t { DS = 0x01, US = 0x02, GND = 0x10 };

struct SampleHeader {
  double t_s = 0.0;
  uint32_t seq = 0;
  bool valid = false;
};

struct TlmPacket {
  SampleHeader hdr{};
  TlmPktType type = TlmPktType::TLM;
  TlmSource src = TlmSource::DS;
  uint16_t apid = 0;
  uint8_t bootCount = 0;
  uint8_t payload[256]{};
  uint16_t payloadLen = 0;
};

struct TlmChannel {
  uint16_t apid = 0;
  uint32_t rateDiv = 10;  // P1 10 Hz @ rg100Hz → 10
  bool enabled = true;
  uint32_t nextTick = 0;
  uint32_t seq = 0;
  double lastTs = 0.0;
  bool haveLastTs = false;
};

enum class TlmSendEvent : uint8_t {
  NONE = 0,
  TIME_NONMONOTONIC = 1,
};

class ITlmTimeSource {
 public:
  virtual ~ITlmTimeSource() = default;
  virtual double now_s() = 0;
};

class ITlmSink {
 public:
  virtual ~ITlmSink() = default;
  virtual void onPacket(const TlmPacket& p) = 0;
};

class ITlmChannelProvider {
 public:
  virtual ~ITlmChannelProvider() = default;
  virtual uint16_t sample(uint16_t apid, uint8_t* buf, uint16_t cap) = 0;
};

class ITlmSendEvents {
 public:
  virtual ~ITlmSendEvents() = default;
  virtual void onTimeNonMonotonic(uint16_t apid, double prev_s, double now_s) = 0;
};

class TlmSendCore {
 public:
  static constexpr size_t kMaxChannels = 16;

  TlmSendCore() = default;

  void setTimeSource(ITlmTimeSource* t) { m_time = t; }
  void setSink(ITlmSink* s) { m_sink = s; }
  void setProvider(ITlmChannelProvider* p) { m_provider = p; }
  void setEventSink(ITlmSendEvents* e) { m_events = e; }

  bool addChannel(uint16_t apid, uint32_t rateDiv);
  bool setEnabled(uint16_t apid, bool en);
  bool setRateDiv(uint16_t apid, uint32_t rateDiv);

  void setSource(TlmSource s) { m_src = s; }
  void setBootCount(uint8_t b) { m_boot = b; }

  void tick();

  size_t channelCount() const { return m_numCh; }
  uint32_t seqOf(uint16_t apid) const;
  uint32_t timeNonMonoCount() const { return m_timeNonMonoCount; }

 private:
  TlmChannel* find(uint16_t apid);

  ITlmTimeSource* m_time = nullptr;
  ITlmSink* m_sink = nullptr;
  ITlmChannelProvider* m_provider = nullptr;
  ITlmSendEvents* m_events = nullptr;

  TlmChannel m_chans[kMaxChannels]{};
  size_t m_numCh = 0;
  uint32_t m_tickCount = 0;
  TlmSource m_src = TlmSource::DS;
  uint8_t m_boot = 0;
  uint32_t m_timeNonMonoCount = 0;
};

}  // namespace Components
#endif
