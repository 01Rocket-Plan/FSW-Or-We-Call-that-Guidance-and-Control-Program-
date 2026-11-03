// ComQueueHostTest：涵蓋 §12.3 全部測試條件
#include "ComQueueCore.hpp"
#include <cassert>
#include <cstdio>
#include <vector>

using namespace Components;

namespace {

struct DropRec : IComQueueEvents {
  int emits = 0;
  uint32_t lastCount = 0;
  ComPriority lastPri = ComPriority::P0;
  void onQueueFullDrop(ComPriority p, uint32_t dropCount) override {
    ++emits;
    lastPri = p;
    lastCount = dropCount;
  }
};

ComFrame mk(uint32_t len) {
  ComFrame f{};
  f.type = 0x01;
  f.apid = 1;
  f.len = len;
  f.data[0] = static_cast<uint8_t>(len & 0xFF);
  return f;
}

void test_fifo_same_tier() {
  ComQueueCore q;
  q.setDepth(ComPriority::P1, 4);
  assert(q.enqueue(ComPriority::P1, mk(1)));
  assert(q.enqueue(ComPriority::P1, mk(2)));
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 1);
  assert(q.dequeue(out) && out.len == 2);
  assert(!q.dequeue(out));
}

void test_cross_priority() {
  ComQueueCore q;
  q.setDepth(ComPriority::P0, 4);
  q.setDepth(ComPriority::P1, 4);
  q.enqueue(ComPriority::P1, mk(10));
  q.enqueue(ComPriority::P0, mk(20));
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 20);
  assert(q.dequeue(out) && out.len == 10);
}

void test_p0_drop_oldest() {
  ComQueueCore q;
  DropRec dr;
  q.setEventSink(&dr);
  q.setDepth(ComPriority::P0, 2);
  assert(q.enqueue(ComPriority::P0, mk(1)));
  assert(q.enqueue(ComPriority::P0, mk(2)));
  assert(q.enqueue(ComPriority::P0, mk(3)));  // drop 1
  assert(q.dropCount(ComPriority::P0) == 1);
  assert(dr.emits == 1);
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 2);
  assert(q.dequeue(out) && out.len == 3);
}

void test_p1_drop_newest() {
  ComQueueCore q;
  q.setDepth(ComPriority::P1, 2);
  assert(q.enqueue(ComPriority::P1, mk(1)));
  assert(q.enqueue(ComPriority::P1, mk(2)));
  assert(!q.enqueue(ComPriority::P1, mk(3)));  // drop newest
  assert(q.dropCount(ComPriority::P1) == 1);
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 1);
  assert(q.dequeue(out) && out.len == 2);
}

void test_high_water_monotonic() {
  ComQueueCore q;
  q.setDepth(ComPriority::P2, 4);
  // 3/4 = 75% → high
  q.enqueue(ComPriority::P2, mk(1));
  q.enqueue(ComPriority::P2, mk(2));
  q.enqueue(ComPriority::P2, mk(3));
  assert(q.highWater());
  assert(q.hwm(ComPriority::P2) == 3);
  ComFrame out{};
  q.dequeue(out);  // 2/4 = 50% — still hold high until <50%
  assert(q.highWater());
  q.dequeue(out);  // 1/4 = 25% → clear
  assert(!q.highWater());
  // re-trigger
  q.enqueue(ComPriority::P2, mk(4));
  q.enqueue(ComPriority::P2, mk(5));
  q.enqueue(ComPriority::P2, mk(6));
  assert(q.highWater());
  assert(q.hwm(ComPriority::P2) >= 3);
}

void test_link_busy_hold() {
  ComQueueCore q;
  q.setDepth(ComPriority::P0, 4);
  q.enqueue(ComPriority::P0, mk(1));
  ComFrame out{};
  assert(!q.nextFrame(true, out));
  assert(q.count(ComPriority::P0) == 1);
  assert(q.nextFrame(false, out) && out.len == 1);
}

void test_drop_event_rate_limit() {
  ComQueueCore q;
  DropRec dr;
  q.setEventSink(&dr);
  q.setDepth(ComPriority::P3, 1);
  q.enqueue(ComPriority::P3, mk(1));
  // burst drops within same window
  for (int i = 0; i < 50; ++i) {
    assert(!q.enqueue(ComPriority::P3, mk(static_cast<uint32_t>(2 + i))));
  }
  assert(dr.emits == 1);  // first only
  assert(q.takeEvent() == ComQueueEvent::QUEUE_FULL_DROP);
  // advance 100 ticks → pending flush of accumulated
  for (uint32_t i = 0; i < ComQueueCore::kDropEventPeriodTicks; ++i) q.tick();
  // one more drop to trigger period check via noteDrop, or tick flush
  assert(!q.enqueue(ComPriority::P3, mk(99)));
  assert(dr.emits >= 2);
  assert(dr.lastCount >= 1);
}

void test_strict_priority_fifo() {
  ComQueueCore q;
  for (uint8_t i = 0; i < 5; ++i) q.setDepth(static_cast<ComPriority>(i), 4);
  q.enqueue(ComPriority::P4, mk(40));
  q.enqueue(ComPriority::P2, mk(20));
  q.enqueue(ComPriority::P2, mk(21));
  q.enqueue(ComPriority::P0, mk(1));
  q.enqueue(ComPriority::P1, mk(10));
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 1);
  assert(q.dequeue(out) && out.len == 10);
  assert(q.dequeue(out) && out.len == 20);
  assert(q.dequeue(out) && out.len == 21);
  assert(q.dequeue(out) && out.len == 40);
}

}  // namespace

int main() {
  test_fifo_same_tier();
  test_cross_priority();
  test_p0_drop_oldest();
  test_p1_drop_newest();
  test_high_water_monotonic();
  test_link_busy_hold();
  test_drop_event_rate_limit();
  test_strict_priority_fifo();
  std::printf("ComQueueHostTest: all 8 tests passed\n");
  return 0;
}
