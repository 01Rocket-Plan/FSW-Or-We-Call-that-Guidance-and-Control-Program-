// SdLoggerCore: queue + seq + monotonic time + drop-newest (host-testable, no F´).
#ifndef RECORDER_SDLOGGERCORE_HPP
#define RECORDER_SDLOGGERCORE_HPP

#include "Components/Recorder/SdLogger/RecordChannels.hpp"
#include "Components/Recorder/SdLogger/SdCardCore.hpp"
#include <cstdint>
#include <deque>

namespace Components {

enum class LoggerDrvState : uint8_t { OFF = 0, INIT = 1, SELFTEST = 2, READY = 3, FAULT = 4 };

class ISdLoggerSink {
 public:
  virtual ~ISdLoggerSink() = default;
  virtual void onQueueFullDrop(uint32_t dropped, uint32_t depth) = 0;
  virtual void onSeqGap(uint32_t expected, uint32_t got) = 0;
  virtual void onTimeNonMonotonic(double prev_s, double now_s) = 0;
  virtual void onSdError(SdMediaStatus status, uint8_t code) = 0;
  virtual void onEnterReady() = 0;
  virtual void onEnterFault(uint8_t reason) = 0;
};

class SdLoggerCore {
 public:
  SdLoggerCore(SdCardCore& card, ISdLoggerSink& sink);

  void setQueueDepth(uint32_t d) { m_queueDepth = d ? d : 1; }
  void setRunning(bool on) { m_running = on; }
  bool running() const { return m_running; }

  //! Init media and enter READY (or FAULT).
  bool start();
  void stop() { m_running = false; }

  //! Enqueue packed record. Never blocks. Drop newest on full.
  bool ingest(const PackedRecord& rec);
  //! Drain queue to media (best effort).
  void flush();

  LoggerDrvState state() const { return m_state; }
  uint32_t dropCount() const { return m_drops; }
  uint32_t writeCount() const { return m_card.writeCount(); }
  uint32_t queueHwm() const { return m_queueHwm; }
  uint32_t queueSize() const { return static_cast<uint32_t>(m_q.size()); }
  uint32_t loggerSeq() const { return m_loggerSeq; }
  uint32_t lastProducerSeq() const { return m_lastProducerSeq; }
  double lastT_s() const { return m_lastT_s; }
  uint32_t sdErrorCount() const { return m_card.errorCount(); }
  uint32_t writePoolHwm() const { return m_card.writePoolHwm(); }
  SdMediaStatus sdStatus() const { return m_card.status(); }

 private:
  void setState(LoggerDrvState s, uint8_t faultReason = 0);
  bool checkSeq(uint16_t channelId, uint32_t producerSeq);
  bool checkTime(double t_s);

  SdCardCore& m_card;
  ISdLoggerSink& m_sink;
  std::deque<PackedRecord> m_q;
  uint32_t m_queueDepth = 64;
  uint32_t m_queueHwm = 0;
  uint32_t m_drops = 0;
  uint32_t m_loggerSeq = 0;
  uint32_t m_lastProducerSeq = 0;
  double m_lastT_s = 0.0;
  bool m_haveTime = false;
  bool m_running = false;
  LoggerDrvState m_state = LoggerDrvState::OFF;
  // per-channel last producer seq; 0xFFFF channels max sparse map via parallel arrays
  bool m_haveSeq[32]{};
  uint32_t m_expectSeq[32]{};
};

}  // namespace Components
#endif
