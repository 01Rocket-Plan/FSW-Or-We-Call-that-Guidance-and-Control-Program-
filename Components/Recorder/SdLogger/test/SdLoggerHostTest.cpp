// Host unit tests for SdCardCore / SdCardSimDevice / SdLoggerCore / record-only layers.
#include "../SdCardCore.hpp"
#include "../SdCardSimDevice.hpp"
#include "../SdLoggerCore.hpp"
#include "../RecordChannels.hpp"
#include <cstdio>
#include <vector>

using namespace Components;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct Sink : ISdLoggerSink {
  int drops = 0, gaps = 0, tnon = 0, sderr = 0, ready = 0, fault = 0;
  uint32_t lastDrop = 0, lastDepth = 0;
  uint32_t gapExp = 0, gapGot = 0;
  double prevT = 0, nowT = 0;
  void onQueueFullDrop(uint32_t d, uint32_t depth) override { drops++; lastDrop = d; lastDepth = depth; }
  void onSeqGap(uint32_t e, uint32_t g) override { gaps++; gapExp = e; gapGot = g; }
  void onTimeNonMonotonic(double p, double n) override { tnon++; prevT = p; nowT = n; }
  void onSdError(SdMediaStatus, uint8_t) override { sderr++; }
  void onEnterReady() override { ready++; }
  void onEnterFault(uint8_t) override { fault++; }
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

int main() {
  testSimRoundtrip();
  testSimFaults();
  testDropNewest();
  testSeqAndTime();
  testLoggerSeqContiguous();
  testRecordLayers();
  std::printf(g_fail ? "SdLoggerHostTest: %d FAILED\n" : "SdLoggerHostTest: ALL PASSED\n", g_fail);
  return g_fail ? 1 : 0;
}
