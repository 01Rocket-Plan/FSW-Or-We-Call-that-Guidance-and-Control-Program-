// FramerTest：7 個測試，覆蓋 §12.2 framer 全部測試條件
// 1) 已知向量組幀後 CRC 正確
// 2) 超長酬載拒絕並發事件
// 3) 欄位偏移與長度與 §2 表一致
// 4) 同步字正確
// 5) 大端序可重現（黃金向量）
// 6) 緩衝不足拒絕並發事件
// 7) APID 超 11 位拒絕並發事件
#include "FramerCore.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace Components;

namespace {
struct EvRecorder : IFramerEvents {
  int count = 0;
  FramerError last = FramerError::NONE;
  uint32_t lastDetail = 0;
  void onFramerError(FramerError code, uint32_t detail) override {
    ++count;
    last = code;
    lastDetail = detail;
  }
};

uint32_t readU32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>((uint16_t(p[0]) << 8) | uint16_t(p[1]));
}
uint64_t readU64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
  return v;
}
}  // namespace

int main() {
  // ---- Test 1: 已知向量 CRC 正確 ----
  {
    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};
    FrameRequest req{};
    req.type = FrameType::TLM;
    req.source = FrameSource::DS;
    req.apid = 0x123;
    req.seq = 0x11223344u;
    req.bootCount = 7;
    req.t_s = 1000.5;
    req.valid = true;
    req.payload = payload;
    req.payloadLen = 4;

    uint8_t out[128];
    FramerCore core;
    FrameResult r = core.build(req, out, sizeof(out));
    assert(r.ok);
    const size_t expectLen = FRAMER_HEADER_BYTES + 4 + FRAMER_TRAILER_CRC;
    assert(r.len == expectLen);

    // 重算 CRC 應與尾端一致
    const size_t crcStart = FRAMER_OFF_LEN;
    const size_t crcLen = r.len - FRAMER_TRAILER_CRC - crcStart;
    uint32_t c = FramerCore::crc32(out + crcStart, crcLen);
    uint32_t tail = readU32(out + r.len - FRAMER_TRAILER_CRC);
    assert(c == tail);
    std::printf("Test1 CRC OK (0x%08X)\n", c);
  }

  // ---- Test 2: 超長酬載拒絕並發事件 ----
  {
    uint8_t payload[16] = {0};
    FrameRequest req{};
    req.apid = 1;
    req.payload = payload;
    req.payloadLen = 16;
    uint8_t out[256];
    EvRecorder ev;
    FramerCore core;
    core.setFrameMax(FRAMER_HEADER_BYTES + 4 + FRAMER_TRAILER_CRC);  // 允許 payload=4
    core.setEventSink(&ev);
    FrameResult r = core.build(req, out, sizeof(out));
    assert(!r.ok);
    assert(ev.count == 1);
    assert(ev.last == FramerError::PAYLOAD_TOO_LARGE);
    std::printf("Test2 PAYLOAD_TOO_LARGE OK (detail=%u)\n", ev.lastDetail);
  }

  // ---- Test 3: 欄位偏移與長度與 §2 一致 ----
  {
    assert(FRAMER_OFF_SYNC == 0);
    assert(FRAMER_OFF_LEN == 4);
    assert(FRAMER_OFF_VER_TYPE == 8);
    assert(FRAMER_OFF_SOURCE == 9);
    assert(FRAMER_OFF_APID == 10);
    assert(FRAMER_OFF_SEQ == 12);
    assert(FRAMER_OFF_BOOT == 16);
    assert(FRAMER_OFF_TS == 17);
    assert(FRAMER_OFF_VALID == 25);
    assert(FRAMER_OFF_PAYLOAD == 26);
    assert(FRAMER_HEADER_BYTES == 26);

    uint8_t payload[2] = {0xAA, 0xBB};
    FrameRequest req{};
    req.type = FrameType::EVT;
    req.source = FrameSource::US;
    req.apid = 0x0A5;
    req.seq = 0xCAFEBABEu;
    req.bootCount = 3;
    req.t_s = 42.25;
    req.valid = false;
    req.payload = payload;
    req.payloadLen = 2;
    uint8_t out[128];
    FramerCore core;
    FrameResult r = core.build(req, out, sizeof(out));
    assert(r.ok);
    assert(out[FRAMER_OFF_VER_TYPE] == static_cast<uint8_t>(FrameType::EVT));
    assert(out[FRAMER_OFF_SOURCE] == static_cast<uint8_t>(FrameSource::US));
    assert(readU16(out + FRAMER_OFF_APID) == 0x0A5);
    assert(readU32(out + FRAMER_OFF_SEQ) == 0xCAFEBABEu);
    assert(out[FRAMER_OFF_BOOT] == 3);
    assert(out[FRAMER_OFF_VALID] == 0);
    assert(readU32(out + FRAMER_OFF_LEN) == 2);
    std::printf("Test3 field offsets OK\n");
  }

  // ---- Test 4: 同步字正確 ----
  {
    uint8_t payload[1] = {0x00};
    FrameRequest req{};
    req.apid = 0;
    req.payload = payload;
    req.payloadLen = 1;
    uint8_t out[64];
    FramerCore core;
    FrameResult r = core.build(req, out, sizeof(out));
    assert(r.ok);
    assert(readU32(out + FRAMER_OFF_SYNC) == 0xDEADBEEFu);
    std::printf("Test4 SYNC OK\n");
  }

  // ---- Test 5: 大端序可重現（黃金向量） ----
  {
    uint8_t payload[2] = {0xDE, 0xAD};
    FrameRequest req{};
    req.type = FrameType::CMD;
    req.source = FrameSource::GROUND;
    req.apid = 0x7FF;
    req.seq = 1;
    req.bootCount = 0;
    req.t_s = 1.0;
    req.valid = true;
    req.payload = payload;
    req.payloadLen = 2;
    uint8_t a[64], b[64];
    FramerCore core;
    core.setFrameMax(64);
    FrameResult ra = core.build(req, a, sizeof(a));
    FrameResult rb = core.build(req, b, sizeof(b));
    assert(ra.ok && rb.ok);
    assert(ra.len == rb.len);
    for (size_t i = 0; i < ra.len; ++i) assert(a[i] == b[i]);
    // 檢查已知欄位大端
    assert(readU32(a + FRAMER_OFF_SYNC) == 0xDEADBEEFu);
    assert(a[FRAMER_OFF_VER_TYPE] == 0x04);
    assert(a[FRAMER_OFF_SOURCE] == 0x10);
    assert(readU16(a + FRAMER_OFF_APID) == 0x07FF);
    assert(readU64(a + FRAMER_OFF_TS) == 0x3FF0000000000000ull);
    std::printf("Test5 golden vector stable OK\n");
  }

  // ---- Test 6: 緩衝不足拒絕並發事件 ----
  {
    uint8_t payload[4] = {1, 2, 3, 4};
    FrameRequest req{};
    req.apid = 1;
    req.payload = payload;
    req.payloadLen = 4;
    uint8_t out[8];
    EvRecorder ev;
    FramerCore core;
    core.setEventSink(&ev);
    FrameResult r = core.build(req, out, sizeof(out));
    assert(!r.ok);
    assert(ev.count == 1);
    assert(ev.last == FramerError::BUFFER_TOO_SMALL);
    std::printf("Test6 BUFFER_TOO_SMALL OK\n");
  }

  // ---- Test 7: APID 超 11 位拒絕並發事件 ----
  {
    uint8_t payload[1] = {0};
    FrameRequest req{};
    req.apid = 0x0800;  // 第 12 位
    req.payload = payload;
    req.payloadLen = 1;
    uint8_t out[64];
    EvRecorder ev;
    FramerCore core;
    core.setEventSink(&ev);
    FrameResult r = core.build(req, out, sizeof(out));
    assert(!r.ok);
    assert(ev.count == 1);
    assert(ev.last == FramerError::BAD_APID);
    assert(ev.lastDetail == 0x0800u);
    std::printf("Test7 BAD_APID OK\n");
  }

  std::printf("ALL FRAMER TESTS PASSED\n");
  return 0;
}
