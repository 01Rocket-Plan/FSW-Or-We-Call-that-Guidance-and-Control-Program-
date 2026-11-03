#include "TlmSendCore.hpp"
#include <cassert>
#include <cstdio>
#include <vector>

using namespace Components;

namespace {

class FakeTime : public ITlmTimeSource {
 public:
  double t = 0.0;
  double now_s() override { return t; }
};

class FakeProvider : public ITlmChannelProvider {
 public:
  explicit FakeProvider(bool hasData = true) : m_has(hasData) {}
  uint16_t sample(uint16_t apid, uint8_t* buf, uint16_t cap) override {
    if (!m_has) return 0;
    if (cap < 4) return 0;
    buf[0] = static_cast<uint8_t>(apid & 0xFF);
    buf[1] = static_cast<uint8_t>((apid >> 8) & 0xFF);
    buf[2] = 0xAB;
    buf[3] = 0xCD;
    return 4;
  }
  void setHasData(bool h) { m_has = h; }
 private:
  bool m_has;
};

class CollectSink : public ITlmSink {
 public:
  std::vector<TlmPacket> pkts;
  void onPacket(const TlmPacket& p) override { pkts.push_back(p); }
};

struct EvRec : ITlmSendEvents {
  int nonmono = 0;
  void onTimeNonMonotonic(uint16_t, double, double) override { ++nonmono; }
};

void test_rate_10hz() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov;
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  assert(core.addChannel(0x010, 10));
  for (int i = 0; i < 100; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  assert(sink.pkts.size() >= 9 && sink.pkts.size() <= 11);
}

void test_seq_contiguous() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov;
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  assert(core.addChannel(200, 5));
  for (int i = 0; i < 50; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  uint32_t expect = 0;
  for (const auto& p : sink.pkts) {
    assert(p.hdr.seq == expect);
    expect++;
  }
  assert(core.seqOf(200) == expect);
}

void test_ts_monotonic() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov;
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  assert(core.addChannel(300, 3));
  for (int i = 0; i < 40; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  double prev = -1.0;
  for (const auto& p : sink.pkts) {
    assert(p.hdr.t_s >= prev);
    prev = p.hdr.t_s;
  }
}

void test_disable_channel() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov;
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  assert(core.addChannel(400, 2));
  for (int i = 0; i < 10; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  size_t before = sink.pkts.size();
  assert(core.setEnabled(400, false));
  for (int i = 10; i < 30; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  assert(sink.pkts.size() == before);
}

void test_no_data() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov(false);
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  assert(core.addChannel(500, 2));
  for (int i = 0; i < 20; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  assert(sink.pkts.empty());
  assert(core.seqOf(500) == 0u);
  prov.setHasData(true);
  for (int i = 20; i < 40; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  assert(!sink.pkts.empty());
  assert(sink.pkts.front().hdr.seq == 0u);
}

void test_multi_apid_seq() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov;
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  assert(core.addChannel(600, 2));
  assert(core.addChannel(601, 4));
  for (int i = 0; i < 40; ++i) {
    time.t = i * 0.01;
    core.tick();
  }
  uint32_t a = 0, b = 0;
  for (const auto& p : sink.pkts) {
    if (p.apid == 600) a++;
    if (p.apid == 601) b++;
  }
  assert(core.seqOf(600) == a);
  assert(core.seqOf(601) == b);
  assert(a > b);
}

void test_dup_and_fields() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov;
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  core.setSource(TlmSource::US);
  core.setBootCount(7);
  assert(core.addChannel(700, 1));
  assert(!core.addChannel(700, 1));
  time.t = 1.0;
  core.tick();
  assert(!sink.pkts.empty());
  const TlmPacket& p = sink.pkts.front();
  assert(p.src == TlmSource::US);
  assert(p.bootCount == 7);
  assert(p.type == TlmPktType::TLM);
  assert(static_cast<uint8_t>(p.type) == 0x01);
}

void test_time_nonmonotonic_event() {
  TlmSendCore core;
  FakeTime time;
  CollectSink sink;
  FakeProvider prov;
  EvRec ev;
  core.setTimeSource(&time);
  core.setSink(&sink);
  core.setProvider(&prov);
  core.setEventSink(&ev);
  assert(core.addChannel(0x010, 1));
  time.t = 10.0;
  core.tick();
  time.t = 9.0;  // go backwards
  core.tick();
  assert(ev.nonmono == 1);
  assert(core.timeNonMonoCount() == 1);
  assert(sink.pkts.size() == 2);
  // 仍送出實際（倒退的）時間
  assert(sink.pkts[1].hdr.t_s == 9.0);
  assert(sink.pkts[1].hdr.seq == 1);
}

}  // namespace

int main() {
  test_rate_10hz();
  test_seq_contiguous();
  test_ts_monotonic();
  test_disable_channel();
  test_no_data();
  test_multi_apid_seq();
  test_dup_and_fields();
  test_time_nonmonotonic_event();
  std::printf("TlmSendHostTest: all 8 tests passed\n");
  return 0;
}
