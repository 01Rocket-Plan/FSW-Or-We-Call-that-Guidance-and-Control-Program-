// ComQueueCore：無線下行優先佇列（與 F´ 無關，可在 host 測試）
// 規則：不 sleep、不讀系統時間；每個 run 最多推進 maxPktsPerRun 幀（≥1）。
// P0..P4：全層滿丟最新（拒收新幀並計數）。EVT_QUEUE_FULL_DROP ≤1 次／100 tick（附累計數）。
// 只留最新（latest-only）：登記的 (層, apid) 在該層最多排一幀，新幀覆蓋尚未送出的舊幀（例：P0 的 GNSS fix）。
// 執行緒安全：F´ 殼以 guarded comIn 持元件 mutex；Core 本身無鎖、無阻塞，
// 呼叫端須在同一把 mutex 下呼叫 enqueue／runEmit（host 測試用 std::mutex 模擬）。
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
  virtual void onQueueFullDrop(ComPriority p, uint32_t dropCount) = 0;
};

class ComQueueCore {
 public:
  static constexpr uint8_t kMaxDepth = 16;
  static constexpr uint32_t kDropEventPeriodTicks = 100;  // 1 s @ rg100Hz
  static constexpr uint8_t kDefaultMaxPktsPerRun = 4;
  static constexpr uint8_t kMaxLatestOnly = 4;

  ComQueueCore();

  void setEventSink(IComQueueEvents* sink) { m_events = sink; }

  bool setDepth(ComPriority p, uint8_t depth);

  // maxPktsPerRun ≥ 1；回 false 表示拒收
  bool setMaxPktsPerRun(uint8_t n);
  uint8_t maxPktsPerRun() const { return m_maxPktsPerRun; }

  // 登記 (層, apid) 為只留最新。同組重複登記視為成功；表滿（kMaxLatestOnly）回 false。
  bool setLatestOnly(ComPriority p, uint16_t apid);
  bool isLatestOnly(ComPriority p, uint16_t apid) const;

  // 入列。只留最新且該層已有同 apid 幀：原位覆蓋（保留排隊位置）、回 true、計入 coalesceCount，不算丟包。
  // 其餘：該層滿則拒收新幀、計數、回 false（各層一致）。
  bool enqueue(ComPriority p, const ComFrame& f);

  // 取最高優先層 FIFO 最舊一幀。
  bool dequeue(ComFrame& out);

  // linkBusy 時不推進；否則 dequeue 一幀（單包路徑）。
  bool nextFrame(bool linkBusy, ComFrame& out);

  // rg100Hz run：更新節流後，在 !linkBusy 時最多吐出 maxPktsPerRun 幀到 out[0..)。
  // 回傳實際寫入筆數；永不一次排空超過上限。busy 時回 0。
  uint8_t runEmit(bool linkBusy, ComFrame* out, uint8_t outCap);

  // 僅推進 drop 節流／高水位（不含發送）。
  void tick();

  bool highWater() const { return m_highWater; }
  ComQueueEvent takeEvent();

  uint8_t depth(ComPriority p) const;
  uint8_t count(ComPriority p) const;
  uint8_t hwm(ComPriority p) const;
  uint32_t dropCount(ComPriority p) const;
  uint32_t dropCountTotal() const;
  uint32_t coalesceCount(ComPriority p) const;
  uint32_t queuedTotal() const;

 private:
  struct Ring {
    ComFrame buf[kMaxDepth]{};
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
    uint8_t depth = kMaxDepth;
    uint8_t hwm = 0;

    void reset();
    bool pushDropNewest(const ComFrame& f);
    bool replaceApid(const ComFrame& f);
    bool pop(ComFrame& out);
  };

  void noteDrop(ComPriority p);
  void updateHighWater();
  void flushDropEvent();

  Ring m_ring[static_cast<uint8_t>(ComPriority::NUM)]{};
  bool m_highWater = false;
  ComQueueEvent m_event = ComQueueEvent::NONE;
  IComQueueEvents* m_events = nullptr;
  uint8_t m_maxPktsPerRun = kDefaultMaxPktsPerRun;

  struct LatestOnlyKey {
    ComPriority pri = ComPriority::P0;
    uint16_t apid = 0;
  };
  LatestOnlyKey m_latestOnly[kMaxLatestOnly]{};
  uint8_t m_latestOnlyN = 0;
  uint32_t m_coalesce[static_cast<uint8_t>(ComPriority::NUM)]{};

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
