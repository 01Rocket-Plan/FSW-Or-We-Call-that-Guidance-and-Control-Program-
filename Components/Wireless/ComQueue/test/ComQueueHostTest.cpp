// ComQueueHostTest：§12.3 + maxPktsPerRun + guarded mutex 模擬
#include "ComQueueCore.hpp"
#include <atomic>
#include <cassert>
#include <cstdio>
#include <mutex>
#include <thread>
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

// P0 放關鍵事件：滿了丟最新，先發生的事件（例如 IMU 失效）要保住
void test_p0_drop_newest_keeps_earliest() {
  ComQueueCore q;
  DropRec dr;
  q.setEventSink(&dr);
  q.setDepth(ComPriority::P0, 2);
  assert(q.enqueue(ComPriority::P0, mk(1)));
  assert(q.enqueue(ComPriority::P0, mk(2)));
  assert(!q.enqueue(ComPriority::P0, mk(3)));
  assert(q.dropCount(ComPriority::P0) == 1);
  assert(dr.emits == 1);
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 1);
  assert(q.dequeue(out) && out.len == 2);
  assert(!q.dequeue(out));
}

ComFrame mkApid(uint16_t apid, uint32_t len) {
  ComFrame f = mk(len);
  f.apid = apid;
  return f;
}

// GNSS fix 在 P0 只留最新：舊座標被覆蓋、保留排隊位置、不算丟包
void test_p0_gnss_latest_only() {
  constexpr uint16_t kGnss = 0x20;  // 測試用 apid（實際值待拓樸定）
  constexpr uint16_t kEvt = 0x10;
  ComQueueCore q;
  DropRec dr;
  q.setEventSink(&dr);
  q.setDepth(ComPriority::P0, 4);
  assert(q.setLatestOnly(ComPriority::P0, kGnss));
  assert(q.enqueue(ComPriority::P0, mkApid(kEvt, 1)));   // 事件 A
  assert(q.enqueue(ComPriority::P0, mkApid(kGnss, 2)));  // fix #1
  assert(q.enqueue(ComPriority::P0, mkApid(kEvt, 3)));   // 事件 B
  assert(q.enqueue(ComPriority::P0, mkApid(kGnss, 4)));  // fix #2 覆蓋 #1
  assert(q.enqueue(ComPriority::P0, mkApid(kGnss, 5)));  // fix #3 覆蓋 #2
  assert(q.count(ComPriority::P0) == 3);
  assert(q.coalesceCount(ComPriority::P0) == 2);
  assert(q.dropCount(ComPriority::P0) == 0);
  assert(dr.emits == 0);
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 1);  // 事件 A
  assert(q.dequeue(out) && out.len == 5 && out.apid == kGnss);  // 最新 fix，在 fix #1 的位置
  assert(q.dequeue(out) && out.len == 3);  // 事件 B
  assert(!q.dequeue(out));
  // 已送出後再來的 fix 是新幀，正常入列
  assert(q.enqueue(ComPriority::P0, mkApid(kGnss, 6)));
  assert(q.count(ComPriority::P0) == 1);
}

// P0 被事件塞滿時：已排隊的 fix 照樣覆蓋；事件仍丟最新；沒排隊的 fix 進不來（事件優先）
void test_p0_full_events_vs_gnss() {
  constexpr uint16_t kGnss = 0x20;
  constexpr uint16_t kEvt = 0x10;
  ComQueueCore q;
  q.setDepth(ComPriority::P0, 2);
  assert(q.setLatestOnly(ComPriority::P0, kGnss));
  assert(!q.isLatestOnly(ComPriority::P1, kGnss));
  assert(q.enqueue(ComPriority::P0, mkApid(kEvt, 1)));
  assert(q.enqueue(ComPriority::P0, mkApid(kGnss, 2)));
  assert(q.enqueue(ComPriority::P0, mkApid(kGnss, 3)));   // 滿了仍可覆蓋
  assert(!q.enqueue(ComPriority::P0, mkApid(kEvt, 4)));   // 事件丟最新
  assert(q.dropCount(ComPriority::P0) == 1);
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 1);
  assert(q.dequeue(out) && out.len == 3);
  ComQueueCore q2;
  q2.setDepth(ComPriority::P0, 1);
  assert(q2.setLatestOnly(ComPriority::P0, kGnss));
  assert(q2.enqueue(ComPriority::P0, mkApid(kEvt, 1)));
  assert(!q2.enqueue(ComPriority::P0, mkApid(kGnss, 2)));  // 不擠掉事件
  assert(q2.dequeue(out) && out.len == 1);
  // 登記表上限
  ComQueueCore q3;
  for (uint16_t a = 0; a < ComQueueCore::kMaxLatestOnly; ++a) assert(q3.setLatestOnly(ComPriority::P0, a));
  assert(q3.setLatestOnly(ComPriority::P0, 0));  // 重複登記 OK
  assert(!q3.setLatestOnly(ComPriority::P0, 99));
}

void test_p1_drop_newest() {
  ComQueueCore q;
  q.setDepth(ComPriority::P1, 2);
  assert(q.enqueue(ComPriority::P1, mk(1)));
  assert(q.enqueue(ComPriority::P1, mk(2)));
  assert(!q.enqueue(ComPriority::P1, mk(3)));
  assert(q.dropCount(ComPriority::P1) == 1);
  ComFrame out{};
  assert(q.dequeue(out) && out.len == 1);
  assert(q.dequeue(out) && out.len == 2);
}

void test_high_water_monotonic() {
  ComQueueCore q;
  q.setDepth(ComPriority::P2, 4);
  q.enqueue(ComPriority::P2, mk(1));
  q.enqueue(ComPriority::P2, mk(2));
  q.enqueue(ComPriority::P2, mk(3));
  assert(q.highWater());
  assert(q.hwm(ComPriority::P2) == 3);
  ComFrame out{};
  q.dequeue(out);
  assert(q.highWater());
  q.dequeue(out);
  assert(!q.highWater());
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
  // runEmit busy → 0
  q.enqueue(ComPriority::P0, mk(2));
  ComFrame buf[4]{};
  assert(q.runEmit(true, buf, 4) == 0);
  assert(q.count(ComPriority::P0) == 1);
}

void test_drop_event_rate_limit() {
  ComQueueCore q;
  DropRec dr;
  q.setEventSink(&dr);
  q.setDepth(ComPriority::P3, 1);
  q.enqueue(ComPriority::P3, mk(1));
  for (int i = 0; i < 50; ++i) {
    assert(!q.enqueue(ComPriority::P3, mk(static_cast<uint32_t>(2 + i))));
  }
  assert(dr.emits == 1);
  assert(q.takeEvent() == ComQueueEvent::QUEUE_FULL_DROP);
  for (uint32_t i = 0; i < ComQueueCore::kDropEventPeriodTicks; ++i) q.tick();
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

void test_max_pkts_per_run_cap() {
  ComQueueCore q;
  assert(q.maxPktsPerRun() == ComQueueCore::kDefaultMaxPktsPerRun);
  assert(!q.setMaxPktsPerRun(0));
  assert(q.setMaxPktsPerRun(3));
  assert(q.maxPktsPerRun() == 3);
  q.setDepth(ComPriority::P1, 16);
  for (uint32_t i = 1; i <= 10; ++i) {
    assert(q.enqueue(ComPriority::P1, mk(i)));
  }
  assert(q.queuedTotal() == 10);
  ComFrame buf[8]{};
  // first run: at most 3
  uint8_t n1 = q.runEmit(false, buf, 8);
  assert(n1 == 3);
  assert(buf[0].len == 1 && buf[1].len == 2 && buf[2].len == 3);
  assert(q.queuedTotal() == 7);
  // subsequent runs preserve FIFO order
  uint8_t n2 = q.runEmit(false, buf, 8);
  assert(n2 == 3);
  assert(buf[0].len == 4 && buf[1].len == 5 && buf[2].len == 6);
  uint8_t n3 = q.runEmit(false, buf, 8);
  assert(n3 == 3);
  assert(buf[0].len == 7 && buf[1].len == 8 && buf[2].len == 9);
  uint8_t n4 = q.runEmit(false, buf, 8);
  assert(n4 == 1);
  assert(buf[0].len == 10);
  assert(q.queuedTotal() == 0);
  assert(q.runEmit(false, buf, 8) == 0);
}

void test_max_pkts_priority_across_runs() {
  ComQueueCore q;
  assert(q.setMaxPktsPerRun(2));
  q.setDepth(ComPriority::P0, 8);
  q.setDepth(ComPriority::P2, 8);
  q.enqueue(ComPriority::P2, mk(20));
  q.enqueue(ComPriority::P2, mk(21));
  q.enqueue(ComPriority::P0, mk(1));
  q.enqueue(ComPriority::P0, mk(2));
  q.enqueue(ComPriority::P0, mk(3));
  ComFrame buf[4]{};
  uint8_t n = q.runEmit(false, buf, 4);
  assert(n == 2);
  assert(buf[0].len == 1 && buf[1].len == 2);  // P0 first
  n = q.runEmit(false, buf, 4);
  assert(n == 2);
  assert(buf[0].len == 3 && buf[1].len == 20);
  n = q.runEmit(false, buf, 4);
  assert(n == 1);
  assert(buf[0].len == 21);
}

// 模擬 F´ guarded 埠的元件 mutex：多執行緒 enqueue + runEmit
void test_concurrent_enqueue_with_mutex() {
  ComQueueCore q;
  q.setDepth(ComPriority::P1, 16);
  assert(q.setMaxPktsPerRun(4));
  std::mutex guard;  // stands in for F´ component mutex on guarded comIn / run
  std::atomic<bool> start{false};
  std::atomic<int> enqueued{0};

  auto producer = [&](uint32_t base) {
    while (!start.load()) { /* spin */ }
    for (int i = 0; i < 40; ++i) {
      ComFrame f = mk(base + static_cast<uint32_t>(i));
      std::lock_guard<std::mutex> lock(guard);
      if (q.enqueue(ComPriority::P1, f)) ++enqueued;
    }
  };

  std::thread t0(producer, 1000);
  std::thread t1(producer, 2000);
  start.store(true);
  t0.join();
  t1.join();

  // Drain under same mutex; never more than maxPktsPerRun per runEmit
  uint32_t got = 0;
  ComFrame buf[8]{};
  for (int round = 0; round < 50; ++round) {
    std::lock_guard<std::mutex> lock(guard);
    uint8_t n = q.runEmit(false, buf, 8);
    assert(n <= q.maxPktsPerRun());
    got += n;
    if (n == 0 && q.queuedTotal() == 0) break;
  }
  assert(got == static_cast<uint32_t>(enqueued.load()));
  assert(q.queuedTotal() == 0);
}

}  // namespace

int main() {
  test_fifo_same_tier();
  test_cross_priority();
  test_p0_drop_newest_keeps_earliest();
  test_p0_gnss_latest_only();
  test_p0_full_events_vs_gnss();
  test_p1_drop_newest();
  test_high_water_monotonic();
  test_link_busy_hold();
  test_drop_event_rate_limit();
  test_strict_priority_fifo();
  test_max_pkts_per_run_cap();
  test_max_pkts_priority_across_runs();
  test_concurrent_enqueue_with_mutex();
  std::printf("ComQueueHostTest: all 13 tests passed\n");
  return 0;
}
