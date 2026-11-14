// SdLoggerCore: queue + seq + monotonic time + drop-newest + image chunks (host-testable, no F´).
// 兩個佇列都是 StaticRing（編譯時固定大小，不用動態記憶體）。
#ifndef RECORDER_SDLOGGERCORE_HPP
#define RECORDER_SDLOGGERCORE_HPP

#include "Components/Recorder/SdLogger/RecordChannels.hpp"
#include "Components/Recorder/SdLogger/SdCardCore.hpp"
#include <cstdint>

namespace Components {

//! 固定大小的環形佇列（Edward 2026-11-14 PR #55：不用動態記憶體）。
//! 容量 N 在編譯時決定，元素放在物件本身裡；執行中不跟系統要記憶體。滿了不覆蓋舊資料（呼叫端丟最新的）。
template <typename T, uint32_t N>
class StaticRing {
 public:
  static_assert(N > 0, "StaticRing capacity must be > 0");
  static constexpr uint32_t capacity() { return N; }
  uint32_t size() const { return m_count; }
  bool empty() const { return m_count == 0; }
  bool full() const { return m_count == N; }
  //! 放到尾端；滿了回 false。
  bool push(const T& v) {
    if (m_count == N) return false;
    m_buf[(m_head + m_count) % N] = v;
    ++m_count;
    return true;
  }
  //! 尾端下一格，直接寫入（省掉先在堆疊上組一份 2 KB 再複製）；滿了回 nullptr。寫完呼叫 commitBack()。
  T* backSlot() { return (m_count == N) ? nullptr : &m_buf[(m_head + m_count) % N]; }
  void commitBack() {
    if (m_count < N) ++m_count;
  }
  //! 最舊的一筆；呼叫前先確認 !empty()。
  const T& front() const { return m_buf[m_head]; }
  void pop() {
    if (m_count == 0) return;
    m_head = (m_head + 1) % N;
    --m_count;
  }

 private:
  T m_buf[N]{};
  uint32_t m_head = 0;   // 最舊一筆的位置
  uint32_t m_count = 0;  // 目前筆數
};

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
  virtual void onImageChunkDrop(uint32_t dropped, uint32_t depth) = 0;
  virtual void onImageFrameGap(uint32_t expected, uint32_t got) = 0;
  virtual void onImageChunkGap(uint32_t frameNumber, uint32_t expected, uint32_t got) = 0;
  virtual void onImageTruncatedSeen(uint32_t frameNumber, uint32_t chunkIndex) = 0;
  //! 深度參數超過編譯時上限 → 改用上限（which：0＝感測佇列，1＝影像佇列）。
  virtual void onQueueDepthClamped(uint8_t which, uint32_t requested, uint32_t maxDepth) = 0;
};

class SdLoggerCore {
 public:
  //! 佇列容量上限（編譯時固定、靜態配置），等於原本兩個深度參數的預設值。
  //! 記憶體：64 × sizeof(PackedRecord) ＋ 8 × sizeof(ImageChunkRecord)，約 64×280 B ＋ 8×2,072 B ≈ 34.5 KB，
  //! 開機就佔著、執行中不變。
  static constexpr uint32_t kMaxRecordQueueDepth = 64;
  static constexpr uint32_t kMaxImageQueueDepth = 8;

  SdLoggerCore(SdCardCore& card, ISdLoggerSink& sink);

  //! 0 → 1；大於上限 → 用上限並通知 sink。回傳實際採用的深度。
  uint32_t setQueueDepth(uint32_t d);
  uint32_t setImageQueueDepth(uint32_t d);
  uint32_t queueDepth() const { return m_queueDepth; }
  uint32_t imageQueueDepth() const { return m_imageQueueDepth; }
  void setRunning(bool on) { m_running = on; }
  bool running() const { return m_running; }

  bool start();
  void stop() { m_running = false; }

  //! Enqueue packed sample. Never blocks. Drop newest on full.
  bool ingest(const PackedRecord& rec);
  //! Enqueue image chunk (≤2048). Never blocks. Drop newest on full.
  bool ingestImageChunk(uint32_t frameNumber, uint32_t chunkIndex, uint8_t flags,
                        const uint8_t* data, uint16_t len, double t_s);
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

  uint32_t imageChunkCount() const { return m_imageChunkCount; }
  uint32_t imageDropCount() const { return m_imageDrops; }
  uint32_t imageFrameCount() const { return m_imageFrameCount; }
  uint32_t imageBytesWritten() const { return m_imageBytesWritten; }
  uint32_t imageQueueHwm() const { return m_imageQueueHwm; }
  uint32_t lastImageFrame() const { return m_lastImageFrame; }
  uint32_t lastImageChunk() const { return m_lastImageChunk; }
  uint32_t imageQueueSize() const { return static_cast<uint32_t>(m_iq.size()); }

 private:
  void setState(LoggerDrvState s, uint8_t faultReason = 0);
  bool checkSeq(uint16_t channelId, uint32_t producerSeq);
  bool checkTime(double t_s);
  void checkImageGaps(uint32_t frameNumber, uint32_t chunkIndex, uint8_t flags);

  SdCardCore& m_card;
  ISdLoggerSink& m_sink;
  StaticRing<PackedRecord, kMaxRecordQueueDepth> m_q;
  StaticRing<ImageChunkRecord, kMaxImageQueueDepth> m_iq;
  uint32_t m_queueDepth = 64;
  uint32_t m_imageQueueDepth = 8;
  uint32_t m_queueHwm = 0;
  uint32_t m_imageQueueHwm = 0;
  uint32_t m_drops = 0;
  uint32_t m_imageDrops = 0;
  uint32_t m_loggerSeq = 0;
  uint32_t m_lastProducerSeq = 0;
  double m_lastT_s = 0.0;
  bool m_haveTime = false;
  bool m_running = false;
  LoggerDrvState m_state = LoggerDrvState::OFF;
  bool m_haveSeq[32]{};
  uint32_t m_expectSeq[32]{};

  // image continuity
  bool m_haveImage = false;
  uint32_t m_curFrame = 0;
  uint32_t m_expectChunk = 0;
  uint32_t m_expectFrame = 0;
  uint32_t m_imageChunkCount = 0;
  uint32_t m_imageFrameCount = 0;
  uint32_t m_imageBytesWritten = 0;
  uint32_t m_lastImageFrame = 0;
  uint32_t m_lastImageChunk = 0;
};

}  // namespace Components
#endif
