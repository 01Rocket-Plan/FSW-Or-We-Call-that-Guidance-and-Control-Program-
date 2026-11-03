// ComQueueCore：無線下行優先佇列（與 F´ 無關，可在 host 測試）
// 規則：不 sleep、不阻塞、不讀系統時間；每個 tick 最多推進一幀。
// P0..P4：P0 滿丟最舊；其餘滿丟最新。EVT_QUEUE_FULL_DROP ≤1 次／100 tick（附累計數）。
#ifndef WIRELESS_COMQUEUECORE_HPP
#define WIRELESS_COMQUEUECORE_HPP

#include <cstdint>

namespace Components {

enum class ComPriority : uint8_t { P0 = 0, P1 = 1, P2 = 2, P3 = 3, P4 = 4, NUM = 5 };

enum class ComQueueEvent : uint8_t {
  NONE = 0,
  QUEUE_FULL_DROP = 1,
  HIGH_WATER = 2,
};

struct ComFrame {
  uint8_t type = 0;
  uint16_t apid = 0;
  uint32_t len = 0;
  uint8_t data[256]{};
};

class IComQueueEvents {
 public:
  virtual ~IComQueueEvents() = default;
  // EVT_QUEUE_FULL_DROP：priority、本秒累計丟棄數
  virtual void onQueueFullDrop(ComPriority p, uint32_t dropCount) = 0;
};

class ComQueueCore {
 public:
  static constexpr uint8_t kMaxDepth = 16;
  static constexpr uint32_t kDropEventPeriodTicks = 100;  // 1 s @ rg100Hz

  ComQueueCore();

  void setEventSink(IComQueueEvents* sink) { m_events = sink; }

  bool setDepth(ComPriority p, uint8_t depth);

  // 入列。P0 滿：丟最舊後插入，回 true；其餘滿：拒收新幀，回 false。皆發 drop 計數。
  bool enqueue(ComPriority p, const ComFrame& f);

  // 取最高優先層 FIFO 最舊一幀。
  bool dequeue(ComFrame& out);

  // linkBusy 時不推進；否則 dequeue 一幀。
  bool nextFrame(bool linkBusy, ComFrame& out);

  // rg100Hz tick：更新 drop 事件節流與高水位。
  void tick();

  bool highWater() const { return m_highWater; }
  ComQueueEvent takeEvent();

  uint8_t depth(ComPriority p) const;
  uint8_t count(ComPriority p) const;
  uint8_t hwm(ComPriority p) const;
  uint32_t dropCount(ComPriority p) const;
  uint32_t dropCountTotal() const;

 private:
  struct Ring {
    ComFrame buf[kMaxDepth]{};
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
    uint8_t depth = kMaxDepth;
    uint8_t hwm = 0;

    void reset();
    bool pushDropOldest(const ComFrame& f);  // returns true if an old frame was dropped
    bool pushDropNewest(const ComFrame& f);  // returns true if new frame was dropped (not stored)
    bool pop(ComFrame& out);
  };

  void noteDrop(ComPriority p);
  void updateHighWater();
  void flushDropEvent();

  Ring m_ring[static_cast<uint8_t>(ComPriority::NUM)]{};
  bool m_highWater = false;
  ComQueueEvent m_event = ComQueueEvent::NONE;
  IComQueueEvents* m_events = nullptr;

  uint32_t m_dropTotal[static_cast<uint8_t>(ComPriority::NUM)]{};
  uint32_t m_dropSinceEmit[static_cast<uint8_t>(ComPriority::NUM)]{};
  uint32_t m_tickCount = 0;
  uint32_t m_lastDropEmitTick = 0;
  bool m_hasEmittedDrop = false;
  bool m_pendingDropEvent = false;
  ComPriority m_pendingDropPri = ComPriority::P0;
};

}  // namespace Components
#endif
