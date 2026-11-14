// Host unit tests for SdCardCore / SdCardSimDevice / SdLoggerCore / record-only layers.
#include "../SdCardCore.hpp"
#include "../SdCardSimDevice.hpp"
#include "../SdLoggerCore.hpp"
#include "../RecordChannels.hpp"
#include <cstdio>
#include <cstring>
#include <vector>

using namespace Components;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct Sink : ISdLoggerSink {
  int drops = 0, gaps = 0, tnon = 0, sderr = 0, ready = 0, fault = 0;
  int imgDrops = 0, imgFrameGaps = 0, imgChunkGaps = 0, imgTrunc = 0;
  uint32_t lastDrop = 0, lastDepth = 0;
  uint32_t gapExp = 0, gapGot = 0;
  uint32_t imgGapExp = 0, imgGapGot = 0, imgFrame = 0, imgChunk = 0;
  double prevT = 0, nowT = 0;
  void onQueueFullDrop(uint32_t d, uint32_t depth) override { drops++; lastDrop = d; lastDepth = depth; }
  void onSeqGap(uint32_t e, uint32_t g) override { gaps++; gapExp = e; gapGot = g; }
  void onTimeNonMonotonic(double p, double n) override { tnon++; prevT = p; nowT = n; }
  void onSdError(SdMediaStatus, uint8_t) override { sderr++; }
  void onEnterReady() override { ready++; }
  void onEnterFault(uint8_t) override { fault++; }
  void onImageChunkDrop(uint32_t d, uint32_t depth) override { imgDrops++; lastDrop = d; lastDepth = depth; }
  void onImageFrameGap(uint32_t e, uint32_t g) override { imgFrameGaps++; imgGapExp = e; imgGapGot = g; }
  void onImageChunkGap(uint32_t frame, uint32_t e, uint32_t g) override {
    imgChunkGaps++; imgFrame = frame; imgGapExp = e; imgGapGot = g;
  }
  void onImageTruncatedSeen(uint32_t frame, uint32_t chunk) override {
    imgTrunc++; imgFrame = frame; imgChunk = chunk;
  }
  void onQueueDepthClamped(uint8_t which, uint32_t requested, uint32_t maxDepth) override {
    clamped++; clampWhich = which; clampReq = requested; clampMax = maxDepth;
  }
  int clamped = 0;
  uint8_t clampWhich = 0xFF;
  uint32_t clampReq = 0, clampMax = 0;
};

static void testSimRoundtrip() {
  SdCardSimDevice dev;
  SdCardCore core(dev);
  CHECK(core.init());
  SdRecordHeader h{static_cast<uint16_t>(RecordChannel::ENV), 7, 3, 1.25};
  const float payload = 42.0f;
  CHECK(core.writeRecord(h, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload)));
  CHECK(core.writeCount() == 1);
  CHECK(dev.bytesWritten() > 0);
  CHECK(core.sync());
}

static void testSimFaults() {
  SdCardSimDevice dev;
  SdCardCore core(dev);
  dev.setFault(SdSimFaultKind::CARD_MISSING);
  CHECK(!core.init());
  CHECK(core.status() == SdMediaStatus::CARD_MISSING);

  SdCardSimDevice dev2;
  SdCardCore core2(dev2);
  CHECK(core2.init());
  dev2.setFault(SdSimFaultKind::WRITE_FAIL);
  SdRecordHeader h{1, 0, 0, 0.0};
  uint8_t b = 0;
  CHECK(!core2.writeRecord(h, &b, 1));
  CHECK(core2.errorCount() >= 1);
}

static void testDropNewest() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  log.setQueueDepth(2);
  CHECK(log.start());
  CHECK(sink.ready == 1);
  PackedRecord r;
  CHECK(packRecord(RecordChannel::ENV, 0, 1.0, nullptr, 0, r));
  CHECK(log.ingest(r));
  r.producerSeq = 1; r.t_s = 1.1;
  CHECK(log.ingest(r));
  r.producerSeq = 2; r.t_s = 1.2;
  CHECK(!log.ingest(r));  // drop newest
  CHECK(sink.drops == 1);
  CHECK(log.dropCount() == 1);
  CHECK(log.queueSize() == 2);
}

static void testSeqAndTime() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  log.setQueueDepth(16);
  CHECK(log.start());
  PackedRecord r;
  CHECK(packRecord(RecordChannel::MPU6050, 10, 5.0, nullptr, 0, r));
  CHECK(log.ingest(r));
  r.producerSeq = 12; r.t_s = 5.1;  // gap: expected 11
  CHECK(log.ingest(r));
  CHECK(sink.gaps == 1);
  CHECK(sink.gapExp == 11 && sink.gapGot == 12);
  r.producerSeq = 13; r.t_s = 4.0;  // time goes backwards
  CHECK(log.ingest(r));
  CHECK(sink.tnon == 1);
}

static void testLoggerSeqContiguous() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  log.setQueueDepth(32);
  CHECK(log.start());
  for (uint32_t i = 0; i < 5; ++i) {
    PackedRecord r;
    CHECK(packRecord(RecordChannel::BARO, i, 1.0 + i * 0.01, nullptr, 0, r));
    CHECK(log.ingest(r));
  }
  log.flush();
  CHECK(log.writeCount() == 5);
  CHECK(log.loggerSeq() == 5);
  CHECK(log.queueSize() == 0);
  CHECK(log.queueHwm() >= 1);
  CHECK(card.writePoolHwm() >= 1);
}

static void testRecordLayers() {
  PackedRecord out;
  Mpu6050RecordPod m{{1, 2, 3}, {4, 5, 6}, 1, 0};
  CHECK(Mpu6050RecordLayer::pack(9, 3.5, m, out));
  CHECK(out.channelId == static_cast<uint16_t>(RecordChannel::MPU6050));
  CHECK(out.producerSeq == 9);
  Dht22RecordPod d{300.0f, 0.4f};
  CHECK(Dht22RecordLayer::pack(1, 1.0, d, out));
  CHECK(out.channelId == static_cast<uint16_t>(RecordChannel::HUM_TEMP));
  GnssRecordPod g{};
  g.lat_rad = 0.1; g.lon_rad = 0.2; g.numSats = 8;
  CHECK(GnssRecordLayer::pack(2, 2.0, g, out));
  CHECK(out.channelId == static_cast<uint16_t>(RecordChannel::GNSS));
}


static void testImageChunkRoundtrip() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  log.setImageQueueDepth(4);
  CHECK(log.start());
  uint8_t payload[2048];
  for (int i = 0; i < 2048; ++i) payload[i] = static_cast<uint8_t>(i & 0xFF);
  // frame 0: chunks 0,1 with END on last
  CHECK(log.ingestImageChunk(0, 0, 0, payload, 2048, 10.0));
  CHECK(log.ingestImageChunk(0, 1, 1 /*END*/, payload, 100, 10.01));
  log.flush();
  CHECK(log.imageChunkCount() == 2);
  CHECK(log.imageFrameCount() == 1);
  CHECK(log.imageBytesWritten() == 2148);  // payload bytes only
  CHECK(log.lastImageFrame() == 0);
  CHECK(log.lastImageChunk() == 1);
  CHECK(log.imageQueueSize() == 0);
  CHECK(card.writeCount() >= 2);
}

static void testImageDropNewest() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  log.setImageQueueDepth(2);
  CHECK(log.start());
  uint8_t b[16] = {};
  CHECK(log.ingestImageChunk(1, 0, 0, b, 16, 1.0));
  CHECK(log.ingestImageChunk(1, 1, 0, b, 16, 1.01));
  CHECK(!log.ingestImageChunk(1, 2, 0, b, 16, 1.02));  // drop newest
  CHECK(sink.imgDrops == 1);
  CHECK(log.imageDropCount() == 1);
  CHECK(log.imageQueueSize() == 2);
}

static void testImageGapsAndTrunc() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  log.setImageQueueDepth(16);
  CHECK(log.start());
  uint8_t b[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  CHECK(log.ingestImageChunk(5, 0, 0, b, 8, 2.0));
  CHECK(log.ingestImageChunk(5, 2, 0, b, 8, 2.01));  // chunk gap: expected 1
  CHECK(sink.imgChunkGaps == 1);
  CHECK(sink.imgGapExp == 1 && sink.imgGapGot == 2);
  CHECK(log.ingestImageChunk(7, 0, 2 /*TRUNCATED*/, b, 8, 2.02));  // frame gap 6 expected
  CHECK(sink.imgFrameGaps == 1);
  CHECK(sink.imgTrunc == 1);
  CHECK(sink.imgFrame == 7);
  CHECK(log.imageQueueDepth() == SdLoggerCore::kMaxImageQueueDepth);  // 16 > 8 → 夾到 8
}

// 靜態環形緩衝：先進先出、繞圈後順序不亂、滿了不覆蓋舊資料。
static void testStaticRingWrap() {
  StaticRing<int, 3> q;
  CHECK(q.empty() && q.capacity() == 3);
  int next = 0, expect = 0;
  for (int round = 0; round < 10; ++round) {  // 每輪放 2 取 2 → head 會繞圈好幾次
    CHECK(q.push(next++));
    CHECK(q.push(next++));
    CHECK(q.size() == 2);
    CHECK(q.front() == expect); q.pop(); ++expect;
    CHECK(q.front() == expect); q.pop(); ++expect;
    CHECK(q.empty());
  }
  CHECK(q.push(100) && q.push(101) && q.push(102));
  CHECK(q.full());
  CHECK(!q.push(103));        // 滿了：不放、不覆蓋
  CHECK(q.backSlot() == nullptr);
  CHECK(q.front() == 100);    // 最舊的還在
  q.pop();
  int* s = q.backSlot();
  CHECK(s != nullptr);
  if (s) { *s = 104; q.commitBack(); }
  CHECK(q.size() == 3);
  CHECK(q.front() == 101); q.pop();
  CHECK(q.front() == 102); q.pop();
  CHECK(q.front() == 104); q.pop();
  CHECK(q.empty());
  q.pop();                    // 空的 pop 不動作
  CHECK(q.size() == 0);
}

// 深度參數：0 → 1；超過編譯時上限 → 用上限並發 QueueDepthClamped；上限內照設。
static void testDepthClamp() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  CHECK(log.queueDepth() == 64 && log.imageQueueDepth() == 8);  // 預設＝上限
  CHECK(log.setQueueDepth(0) == 1 && sink.clamped == 0);
  CHECK(log.setQueueDepth(64) == 64 && sink.clamped == 0);
  CHECK(log.setQueueDepth(100) == SdLoggerCore::kMaxRecordQueueDepth);
  CHECK(sink.clamped == 1 && sink.clampWhich == 0 && sink.clampReq == 100 && sink.clampMax == 64);
  CHECK(log.setImageQueueDepth(13) == SdLoggerCore::kMaxImageQueueDepth);
  CHECK(sink.clamped == 2 && sink.clampWhich == 1 && sink.clampReq == 13 && sink.clampMax == 8);
  CHECK(log.setImageQueueDepth(5) == 5 && sink.clamped == 2);
  // 滿上限 64 筆後第 65 筆丟掉
  CHECK(log.start());
  PackedRecord r;
  CHECK(packRecord(RecordChannel::BARO, 0, 1.0, nullptr, 0, r));
  for (uint32_t i = 0; i < 64; ++i) { r.producerSeq = i; CHECK(log.ingest(r)); }
  r.producerSeq = 64;
  CHECK(!log.ingest(r));
  CHECK(log.queueSize() == 64 && log.dropCount() == 1);
}

// 影像環形緩衝繞圈：寫進 SD 的內容和順序要對（繞圈不能讀到舊格子），沒給 data 的那塊要是全 0。
static void testImageRingWrapContent() {
  SdCardSimDevice dev;
  SdCardCore card(dev);
  Sink sink;
  SdLoggerCore log(card, sink);
  CHECK(log.start());
  uint8_t b[64];
  uint32_t written = 0;
  for (uint32_t frame = 0; frame < 5; ++frame) {  // 5 幀 × 3 塊，每幀 flush → 環形緩衝繞圈
    for (uint32_t c = 0; c < 3; ++c) {
      for (int i = 0; i < 64; ++i) b[i] = static_cast<uint8_t>(frame * 31 + c * 7 + i);
      CHECK(log.ingestImageChunk(frame, c, c == 2 ? 1 : 0, b, 64, 1.0 + frame + c * 0.01));
    }
    log.flush();
    written += 3;
    CHECK(log.imageChunkCount() == written && log.imageQueueSize() == 0);
  }
  CHECK(log.imageFrameCount() == 5 && sink.imgChunkGaps == 0 && sink.imgFrameGaps == 0);
  // 最後一塊（frame 4, chunk 2）的內容要出現在 SD 影像裡
  for (int i = 0; i < 64; ++i) b[i] = static_cast<uint8_t>(4 * 31 + 2 * 7 + i);
  const std::vector<uint8_t>& img = dev.image();
  bool found = false;
  for (size_t p = 0; p + 64 <= img.size() && !found; ++p) {
    found = std::memcmp(img.data() + p, b, 64) == 0;
  }
  CHECK(found);
  // 塞滿 8 格（上一輪留下資料）再 flush，接著送一塊「有長度沒 data」→ 那格要清 0，不能是舊資料
  for (uint32_t c = 0; c < 8; ++c) {
    for (int i = 0; i < 64; ++i) b[i] = 0xAB;
    CHECK(log.ingestImageChunk(5, c, 0, b, 64, 7.0 + c * 0.01));
  }
  CHECK(!log.ingestImageChunk(5, 8, 0, b, 64, 7.1));  // 第 9 塊：滿了丟最新
  log.flush();
  const size_t before = dev.image().size();
  CHECK(log.ingestImageChunk(5, 9, 1, nullptr, 64, 7.2));
  log.flush();
  const std::vector<uint8_t>& img2 = dev.image();
  CHECK(img2.size() > before + 64);
  bool allZero = true;
  for (size_t p = img2.size() - 64; p < img2.size(); ++p) allZero = allZero && img2[p] == 0;
  CHECK(allZero);
}


int main() {
  testSimRoundtrip();
  testSimFaults();
  testDropNewest();
  testSeqAndTime();
  testLoggerSeqContiguous();
  testRecordLayers();
  testImageChunkRoundtrip();
  testImageDropNewest();
  testImageGapsAndTrunc();
  testStaticRingWrap();
  testDepthClamp();
  testImageRingWrapContent();
  std::printf(g_fail ? "SdLoggerHostTest: %d FAILED\n" : "SdLoggerHostTest: ALL PASSED\n", g_fail);
  return g_fail ? 1 : 0;
}
