// SdCardSimTimingTest：SdCardSimDevice 有限寫入速度（SIL 測試發現，見 SSS_Mission_Status #4／#5）。
// 預設 10 MB/s（SD Speed Class 10／UHS U1 最低連續寫入；1 MB/s = 1 000 000 B/s）、10 ms 視窗。
#include "Components/Recorder/SdLogger/SdCardCore.hpp"
#include "Components/Recorder/SdLogger/SdCardSimDevice.hpp"
#include "Components/Recorder/SdLogger/SdLoggerCore.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

using namespace Components;

TEST(SdCardSimTiming, DefaultsAreClass10And10msWindow) {
  SdCardSimDevice dev;
  EXPECT_EQ(dev.writeBytesPerSec(), 10000000u);
  EXPECT_EQ(dev.windowUs(), 10000u);
  EXPECT_EQ(dev.windowBudget(), 100000);  // 10 MB/s × 10 ms
  EXPECT_EQ(+SdCardSimDevice::kClass2_Bps, 2000000u);
  EXPECT_EQ(+SdCardSimDevice::kClass4_Bps, 4000000u);
  EXPECT_EQ(+SdCardSimDevice::kClass6_Bps, 6000000u);
  EXPECT_EQ(+SdCardSimDevice::kU3_Bps, 30000000u);
  EXPECT_EQ(+SdCardSimDevice::kSdhcWriteTimeout_us, 250000u);
}

TEST(SdCardSimTiming, WindowBudgetLimitsBytesPerWindow) {
  SdCardSimDevice dev;
  ASSERT_TRUE(dev.open());
  static uint8_t buf[10000];
  std::memset(buf, 0x5A, sizeof(buf));
  // 一個視窗 100 000 B：10 筆 10 000 B 剛好用完，第 11 筆要等下一個視窗
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(dev.append(buf, sizeof(buf))) << i;
  }
  EXPECT_FALSE(dev.append(buf, sizeof(buf)));
  EXPECT_EQ(dev.status(), SdMediaStatus::BUSY);
  EXPECT_EQ(dev.bytesWritten(), 100000u);
  EXPECT_EQ(dev.busyRejects(), 1u);
  dev.advance();
  EXPECT_EQ(dev.status(), SdMediaStatus::OK);
  EXPECT_TRUE(dev.append(buf, sizeof(buf)));
  EXPECT_EQ(dev.bytesWritten(), 110000u);
}

TEST(SdCardSimTiming, LongRunAverageEqualsConfiguredSpeed) {
  SdCardSimDevice dev;
  dev.setWriteBytesPerSec(SdCardSimDevice::kClass4_Bps);  // 4 MB/s → 40 000 B / 10 ms
  ASSERT_TRUE(dev.open());
  static uint8_t buf[3000];  // 不整除視窗額度，驗透支會扣回
  uint32_t windows = 0;
  for (windows = 0; windows < 100; ++windows) {  // 1 s
    while (dev.append(buf, sizeof(buf))) {
    }
    dev.advance();
  }
  // 1 s × 4 MB/s = 4 000 000 B；透支最多一筆（3000 B）
  EXPECT_GE(dev.bytesWritten(), 4000000u);
  EXPECT_LE(dev.bytesWritten(), 4000000u + sizeof(buf));
}

TEST(SdCardSimTiming, OversizeRecordDoesNotDeadlock) {
  SdCardSimDevice dev;
  dev.setWriteBytesPerSec(100000);  // 100 kB/s → 1000 B / 10 ms，小於一筆 2 KB 影像
  ASSERT_TRUE(dev.open());
  static uint8_t buf[2080];
  EXPECT_TRUE(dev.append(buf, sizeof(buf)));   // 透支
  EXPECT_FALSE(dev.append(buf, sizeof(buf)));  // 額度 < 0
  dev.advance();                               // −1080 + 1000 = −80
  EXPECT_FALSE(dev.append(buf, sizeof(buf)));
  dev.advance();                               // +920
  EXPECT_TRUE(dev.append(buf, sizeof(buf)));
}

TEST(SdCardSimTiming, InjectedWorstCaseBusy250ms) {
  SdCardSimDevice dev;
  ASSERT_TRUE(dev.open());
  uint8_t b[16] = {};
  const uint32_t n = SdCardSimDevice::kSdhcWriteTimeout_us / dev.windowUs();  // 25 個 10 ms 視窗
  EXPECT_EQ(n, 25u);
  dev.injectBusyWindows(n);
  for (uint32_t i = 0; i < n; ++i) {
    EXPECT_FALSE(dev.append(b, sizeof(b))) << i;
    EXPECT_EQ(dev.status(), SdMediaStatus::BUSY);
    dev.advance();
  }
  EXPECT_TRUE(dev.append(b, sizeof(b)));
}

TEST(SdCardSimTiming, UnlimitedWhenZero) {
  SdCardSimDevice dev;
  dev.setWriteBytesPerSec(0);
  ASSERT_TRUE(dev.open());
  static uint8_t buf[50000];
  for (int i = 0; i < 10; ++i) EXPECT_TRUE(dev.append(buf, sizeof(buf)));
}

namespace {
class NullSink : public ISdLoggerSink {
 public:
  void onQueueFullDrop(uint32_t, uint32_t) override {}
  void onSeqGap(uint32_t, uint32_t) override {}
  void onTimeNonMonotonic(double, double) override {}
  void onImageChunkDrop(uint32_t, uint32_t) override {}
  void onImageFrameGap(uint32_t, uint32_t) override {}
  void onImageChunkGap(uint32_t, uint32_t, uint32_t) override {}
  void onImageTruncatedSeen(uint32_t, uint32_t) override {}
  void onQueueDepthClamped(uint8_t, uint32_t, uint32_t) override {}
  void onSdError(SdMediaStatus s, uint8_t code) override {
    ++errors;
    if (s == SdMediaStatus::BUSY) ++busy;
    lastCode = code;
  }
  void onEnterReady() override {}
  void onEnterFault(uint8_t) override { ++faults; }
  uint32_t busy = 0;
  uint32_t errors = 0;
  uint32_t faults = 0;
  uint8_t lastCode = 0;
};
}  // namespace

TEST(SdCardSimTiming, LoggerKeepsChunkQueuedWhileBusy) {
  // 慢卡：SdLoggerCore 遇到 BUSY 把影像塊留在佇列、下一個視窗再寫，不進 FAULT、不丟塊
  SdCardSimDevice dev;
  dev.setWriteBytesPerSec(204800);  // 2048 B / 10 ms
  SdCardCore card(dev);
  NullSink sink;
  SdLoggerCore log(card, sink);
  ASSERT_TRUE(log.start());
  static uint8_t chunk[2048];
  for (uint32_t c = 0; c < 4; ++c) {
    const uint8_t flags = (c == 3) ? 1u : 0u;
    ASSERT_TRUE(log.ingestImageChunk(0, c, flags, chunk, sizeof(chunk), 0.01 * c));
  }
  for (int w = 0; w < 10 && log.imageChunkCount() < 4; ++w) {
    log.flush();
    dev.advance();
  }
  EXPECT_EQ(log.imageChunkCount(), 4u);
  EXPECT_EQ(log.imageDropCount(), 0u);
  EXPECT_GT(log.sdBusyCount(), 0u);  // BUSY 有發生，記在遙測
  EXPECT_EQ(sink.errors, 0u);        // 但不發 SdError（背壓不是錯誤）
  EXPECT_EQ(log.sdErrorCount(), 0u);
  EXPECT_EQ(sink.faults, 0u);
}

TEST(SdCardSimTiming, SlowCardNoEventFlood) {
  // 慢卡 1 s：紀錄＋影像兩個佇列持續塞滿，每 tick 都會遇到 BUSY，但每 tick 都有進度 → 0 個 SdError
  SdCardSimDevice dev;
  dev.setWriteBytesPerSec(204800);  // 2048 B / 10 ms，小於每 tick 的輸入量
  SdCardCore card(dev);
  NullSink sink;
  SdLoggerCore log(card, sink);
  ASSERT_TRUE(log.start());
  static uint8_t chunk[2048];
  PackedRecord r{};
  r.channelId = 1;
  r.len = 32;
  uint32_t seq = 0;
  for (uint32_t tick = 0; tick < 100; ++tick) {
    for (int k = 0; k < 4; ++k) {
      r.producerSeq = seq++;
      r.t_s = 0.01 * tick;
      (void)log.ingest(r);
    }
    (void)log.ingestImageChunk(tick, 0, 1, chunk, sizeof(chunk), 0.01 * tick);
    log.flush();
    dev.advance();
  }
  EXPECT_GE(log.sdBusyCount(), 90u);  // 幾乎每 tick 都 BUSY
  EXPECT_LE(log.sdBusyCount(), 100u);  // 每 tick 最多 +1
  EXPECT_EQ(sink.errors, 0u);          // 舊行為是每 tick 1～2 個 SdError
  EXPECT_EQ(log.sdErrorCount(), 0u);
  EXPECT_EQ(sink.faults, 0u);
  EXPECT_EQ(log.state(), LoggerDrvState::READY);
}

TEST(SdCardSimTiming, StuckBusyEmitsOneEventAfter250ms) {
  // 卡完全卡住 50 個視窗（500 ms）：第 26 個沒進度的 tick（> 250 ms）發一次 SdError(BUSY, 4)，之後不再發
  SdCardSimDevice dev;
  SdCardCore card(dev);
  NullSink sink;
  SdLoggerCore log(card, sink);
  ASSERT_TRUE(log.start());
  EXPECT_EQ(log.busyTimeoutTicks(), 25u);
  PackedRecord r{};
  r.channelId = 1;
  r.len = 16;
  ASSERT_TRUE(log.ingest(r));
  dev.injectBusyWindows(50);
  for (uint32_t tick = 1; tick <= 50; ++tick) {
    log.flush();
    dev.advance();
    EXPECT_EQ(sink.errors, tick <= 25u ? 0u : 1u) << "tick " << tick;
  }
  EXPECT_EQ(sink.busy, 1u);
  EXPECT_EQ(sink.lastCode, +SdLoggerCore::kSdErrBusyTimeout);  // C++14：+ 避免 ODR-use
  EXPECT_EQ(log.sdBusyCount(), 50u);
  EXPECT_EQ(sink.faults, 0u);
  // 卡恢復：寫進去、計數歸零；再卡住要重新數 25 tick 才會再發
  log.flush();
  EXPECT_EQ(log.writeCount(), 1u);
  EXPECT_EQ(log.busyStallTicks(), 0u);
  ASSERT_TRUE(log.ingest(r));
  dev.injectBusyWindows(30);
  for (uint32_t tick = 1; tick <= 26; ++tick) {
    log.flush();
    dev.advance();
  }
  EXPECT_EQ(sink.errors, 2u);
}

TEST(SdCardSimTiming, CardMissingStillAnError) {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  NullSink sink;
  SdLoggerCore log(card, sink);
  ASSERT_TRUE(log.start());
  PackedRecord r{};
  r.channelId = 1;
  r.len = 16;
  ASSERT_TRUE(log.ingest(r));
  dev.setFault(SdSimFaultKind::CARD_MISSING);
  log.flush();
  EXPECT_EQ(sink.errors, 1u);
  EXPECT_EQ(log.sdErrorCount(), 1u);
  EXPECT_EQ(sink.faults, 1u);
  EXPECT_EQ(log.sdBusyCount(), 0u);
}

// Edward 提供的數字：每 10 ms 10 萬 B（＝ 10 000 000 B/s，同 SD Class 10／U1）。規格值、非實測。
// 預設設定下：每個 10 ms 視窗剛好寫進 100 000 B，多 1 B 就 BUSY；100 個視窗（1 s）剛好 10 000 000 B。
TEST(SdCardSimTiming, EdwardFigure100kBPer10ms) {
  SdCardSimDevice dev;
  ASSERT_TRUE(dev.open());
  static uint8_t rec[1000];  // 整除視窗額度 → 不透支，總量要剛好相等
  std::memset(rec, 0xA5, sizeof(rec));
  uint8_t one = 0;
  uint64_t perWindowMax = 0;
  uint64_t perWindowMin = UINT64_MAX;
  for (uint32_t w = 0; w < 100; ++w) {
    const size_t before = dev.bytesWritten();
    while (dev.append(rec, sizeof(rec))) {
    }
    EXPECT_FALSE(dev.append(&one, 1)) << w;  // 額度用完：多 1 B 也不行
    EXPECT_EQ(dev.status(), SdMediaStatus::BUSY);
    const uint64_t got = dev.bytesWritten() - before;
    perWindowMax = (got > perWindowMax) ? got : perWindowMax;
    perWindowMin = (got < perWindowMin) ? got : perWindowMin;
    dev.advance();
  }
  EXPECT_EQ(perWindowMin, 100000u);
  EXPECT_EQ(perWindowMax, 100000u);
  EXPECT_EQ(dev.bytesWritten(), 10000000u);
}
