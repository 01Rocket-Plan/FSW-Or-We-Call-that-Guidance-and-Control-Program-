// RfAirtimeTest：issue #60 第 2、3、5 項
//   - LoraTimeOnAir 整數公式 ＝ Semtech SX1276/77/78 datasheet §4.1.1.7（與 Semtech LoRa Calculator 同一公式）
//   - RfDriverCore TX 完成：模型（數 tick）／TxDone IRQ／逾時
//   - 預設參數下 255 B 連續下行的實際吞吐量
//
// 期望值都是手算（算式寫在註解），不是抄程式輸出。
// 共通：Tsym = 2^SF / BW；T = (npre + 4.25 + npayload)·Tsym；
//       npayload = 8 + max(ceil((8PL − 4SF + 28 + 16CRC − 20IH) / (4(SF − 2DE)))·(CR+4), 0)
#include "Components/Wireless/RfDriver/LoraTimeOnAir.hpp"
#include "Components/Wireless/RfDriver/RfDriverCore.hpp"
#include "Components/Wireless/RfDriverSim/RfDriverSimDevice.hpp"
#include <gtest/gtest.h>
#include <vector>

using namespace Components;

namespace {

LoraModemConfig cfg(uint8_t sf, uint32_t bw_Hz, bool crc = true, LoraLdro ldro = LoraLdro::AUTO) {
  LoraModemConfig c{};
  c.sf = sf;
  c.bw_Hz = bw_Hz;
  c.crcOn = crc;
  c.ldro = ldro;
  return c;
}

class NullSink : public IRfSink {
 public:
  void onStateChange(DrvState, DrvState) override {}
  void onEvent(RfEvent ev, uint32_t) override { events.push_back(ev); }
  void onUplink(const uint8_t*, uint32_t) override {}
  void onComStatus(bool ready) override { comReady = ready; }
  bool has(RfEvent e) const {
    for (RfEvent x : events) if (x == e) return true;
    return false;
  }
  std::vector<RfEvent> events;
  bool comReady = false;
};

//! 永遠有下一包、每包 len 位元組（模擬 ComQueue 一直塞滿）
class InfiniteTxQueue : public IRfTxQueue {
 public:
  explicit InfiniteTxQueue(uint32_t len) : m_len(len) {}
  uint32_t popFrame(uint8_t* buf, uint32_t maxLen) override {
    if (m_limit == 0u || m_len > maxLen) return 0;
    if (m_limit != 0xFFFFFFFFu) --m_limit;
    for (uint32_t i = 0; i < m_len; ++i) buf[i] = static_cast<uint8_t>(i);
    ++popped;
    return m_len;
  }
  void limit(uint32_t n) { m_limit = n; }
  uint32_t popped = 0;

 private:
  uint32_t m_len;
  uint32_t m_limit = 0xFFFFFFFFu;
};

//! 真機替身：TxDone IRQ 由測試決定何時觸發
class IrqBus : public RfDriverSimDevice, public IRfTxDoneSource {
 public:
  bool takeTxDone() override {
    const bool f = irq;
    irq = false;
    return f;
  }
  void abortTx() override { ++abortCalls; }
  bool irq = false;
  uint32_t abortCalls = 0;
};

void toReady(RfDriverCore& drv, RfDriverSimDevice& bus, double& t, double dt) {
  drv.start();
  for (int i = 0; i < 20 && drv.state() != DrvState::READY; ++i) {
    bus.advance();
    drv.tick(t);
    t += dt;
  }
  ASSERT_EQ(drv.state(), DrvState::READY);
}

}  // namespace

// ---------------- 第 2 項：公式參考值（PL = 255，前導 8，顯式標頭，CRC 開，CR 4/5）----------------

TEST(RfAirtime, Ref_SF7_BW125_255B) {
  // Tsym = 128/125k = 1.024 ms；DE = 0；num = 2040 − 28 + 28 + 16 = 2056；den = 28；ceil(73.43) = 74
  // npayload = 8 + 74·5 = 378；T = (12.25 + 378)·1.024 = 399.616 ms
  const LoraModemConfig c = cfg(7, 125000);
  EXPECT_EQ(loraSymbolTimeUs(c), 1024u);
  EXPECT_EQ(loraLowDataRateOptimize(c), 0u);
  EXPECT_EQ(loraPayloadSymbols(c, 255), 378u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 399616u);
}

TEST(RfAirtime, Ref_SF9_BW125_255B) {
  // Tsym = 4.096 ms；num = 2040 − 36 + 44 = 2048；den = 36；ceil(56.89) = 57；npayload = 293
  // T = 305.25·4.096 = 1250.304 ms
  const LoraModemConfig c = cfg(9, 125000);
  EXPECT_EQ(loraPayloadSymbols(c, 255), 293u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 1250304u);
}

TEST(RfAirtime, Ref_SF10_BW125_255B_LdroAutoOff) {
  // Tsym = 8.192 ms ≤ 16 ms → DE = 0；num = 2040 − 40 + 44 = 2044；den = 40；ceil(51.1) = 52；npayload = 268
  // T = 280.25·8.192 = 2295.808 ms
  const LoraModemConfig c = cfg(10, 125000);
  EXPECT_EQ(loraLowDataRateOptimize(c), 0u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 2295808u);
}

TEST(RfAirtime, Ref_SF11_BW125_255B_LdroOnAndOff) {
  // Tsym = 16.384 ms > 16 ms → AUTO 開 DE = 1
  // num = 2040 − 44 + 44 = 2040
  // LDRO 開：den = 4·(11−2) = 36；ceil(56.67) = 57；npayload = 8 + 285 = 293；T = 305.25·16.384 = 5001.216 ms
  // LDRO 關：den = 44；ceil(46.36) = 47；npayload = 8 + 235 = 243；T = 255.25·16.384 = 4182.016 ms
  LoraModemConfig c = cfg(11, 125000);
  EXPECT_EQ(loraSymbolTimeUs(c), 16384u);
  EXPECT_EQ(loraLowDataRateOptimize(c), 1u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 5001216u);
  c.ldro = LoraLdro::ON;
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 5001216u);
  c.ldro = LoraLdro::OFF;
  EXPECT_EQ(loraLowDataRateOptimize(c), 0u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 4182016u);
}

TEST(RfAirtime, Ref_SF12_BW125_255B_LdroOnAndOff) {
  // Tsym = 32.768 ms → AUTO 開；num = 2040 − 48 + 44 = 2036
  // LDRO 開：den = 40；ceil(50.9) = 51；npayload = 8 + 255 = 263；T = 275.25·32.768 = 9019.392 ms
  // LDRO 關：den = 48；ceil(42.42) = 43；npayload = 8 + 215 = 223；T = 235.25·32.768 = 7708.672 ms
  LoraModemConfig c = cfg(12, 125000);
  EXPECT_EQ(loraLowDataRateOptimize(c), 1u);
  EXPECT_EQ(loraPayloadSymbols(c, 255), 263u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 9019392u);
  c.ldro = LoraLdro::OFF;
  EXPECT_EQ(loraPayloadSymbols(c, 255), 223u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 7708672u);
}

TEST(RfAirtime, Ref_BW250_and_BW500_255B) {
  // SF7/250k：Tsym = 0.512 ms，npayload 同 SF7/125k = 378 → 390.25·0.512 = 199.808 ms
  // SF7/500k：Tsym = 0.256 ms → 99.904 ms
  // SF12/250k：Tsym = 16.384 ms > 16 → AUTO 開；npayload = 263（同 SF12/125k LDRO 開）→ 275.25·16.384 = 4509.696 ms
  EXPECT_EQ(loraTimeOnAirUs(cfg(7, 250000), 255), 199808u);
  EXPECT_EQ(loraTimeOnAirUs(cfg(7, 500000), 255), 99904u);
  const LoraModemConfig c = cfg(12, 250000);
  EXPECT_EQ(loraLowDataRateOptimize(c), 1u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 4509696u);
}

TEST(RfAirtime, Ref_BW7_8k_UsesExactBandwidth) {
  // 7.8 kHz 實際 = 500k/64 = 7812.5 Hz → SF7 Tsym = 128/7812.5 = 16.384 ms > 16 → AUTO 開 LDRO
  // num = 2056；den = 4·(7−2) = 20；ceil(102.8) = 103；npayload = 8 + 515 = 523；T = 535.25·16.384 = 8769.536 ms
  const LoraModemConfig c = cfg(7, 7800);
  EXPECT_EQ(loraSymbolTimeUs(c), 16384u);
  EXPECT_EQ(loraLowDataRateOptimize(c), 1u);
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 8769536u);
}

TEST(RfAirtime, Ref_SF6_ImplicitHeader) {
  // SF6 必須隱式標頭；Tsym = 0.512 ms；num = 2040 − 24 + 28 + 16 − 20 = 2040；den = 24；= 85
  // npayload = 8 + 425 = 433；T = 445.25·0.512 = 227.968 ms
  LoraModemConfig c = cfg(6, 125000);
  EXPECT_FALSE(loraConfigValid(c));
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 0u);
  c.explicitHeader = false;
  EXPECT_EQ(loraTimeOnAirUs(c, 255), 227968u);
}

TEST(RfAirtime, Ref_PublicCalculator_64B) {
  // LoRaWAN 51 B 應用酬載＋13 B 標頭 = PL 64（常見的公開空中時間計算器範例）
  // SF7：num = 512 − 28 + 44 = 528；ceil(18.86) = 19；npayload = 103；T = 115.25·1.024 = 118.016 ms
  // SF12（LDRO 開）：num = 512 − 48 + 44 = 508；den = 40；ceil(12.7) = 13；npayload = 73；T = 85.25·32.768 = 2793.472 ms
  EXPECT_EQ(loraTimeOnAirUs(cfg(7, 125000), 64), 118016u);
  EXPECT_EQ(loraTimeOnAirUs(cfg(12, 125000), 64), 2793472u);
}

TEST(RfAirtime, RfParamsDefaultsAndValidation) {
  RfParams p;
  EXPECT_EQ(p.spreadingFactor, 7u);
  EXPECT_EQ(p.bandwidth, LoraBw::BW_125K);
  EXPECT_EQ(p.codingRate, LoraCr::CR_4_5);
  EXPECT_EQ(p.preambleLen, 8u);
  EXPECT_TRUE(p.crcOn);
  EXPECT_FALSE(p.implicitHeader);
  EXPECT_EQ(p.lowDataRateOptimize, LoraLdro::AUTO);
  EXPECT_EQ(p.tickPeriod_us, 10000u);
  EXPECT_TRUE(rfParamsValid(p));
  EXPECT_EQ(loraTimeOnAirUs(loraConfigFromRfParams(p), 255), 399616u);

  RfParams q = p;
  q.spreadingFactor = 5;
  EXPECT_FALSE(rfParamsValid(q));
  q.spreadingFactor = 13;
  EXPECT_FALSE(rfParamsValid(q));
  q = p;
  q.spreadingFactor = 6;  // SF6＋顯式 ✗
  EXPECT_FALSE(rfParamsValid(q));
  q.implicitHeader = true;
  EXPECT_TRUE(rfParamsValid(q));
  q = p;
  q.bandwidth = static_cast<LoraBw>(10);
  EXPECT_FALSE(rfParamsValid(q));
  q = p;
  q.codingRate = static_cast<LoraCr>(0);
  EXPECT_FALSE(rfParamsValid(q));
  q = p;
  q.tickPeriod_us = 0;
  EXPECT_FALSE(rfParamsValid(q));
  // 全部 10 種頻寬都合法，Hz ↔ 編碼可來回
  for (uint8_t b = 0; b <= 9; ++b) {
    q = p;
    q.bandwidth = static_cast<LoraBw>(b);
    EXPECT_TRUE(rfParamsValid(q)) << int(b);
    LoraBw back = LoraBw::BW_500K;
    EXPECT_TRUE(loraBwFromHz(loraBwHz(q.bandwidth), back)) << int(b);
    EXPECT_EQ(back, q.bandwidth);
  }
  LoraBw untouched = LoraBw::BW_250K;
  EXPECT_FALSE(loraBwFromHz(100000, untouched));
  EXPECT_EQ(untouched, LoraBw::BW_250K);
}

TEST(RfAirtime, NoOverflowAtExtremes) {
  // 最長：SF12／7.8 kHz／4:8／前導 65535／PL 255 —— 只要求不溢位、單調
  LoraModemConfig c = cfg(12, 7800);
  c.cr = 4;
  c.preambleLen = 65535;
  const uint64_t big = loraTimeOnAirUs(c, 255);
  // Tsym = 2^13·64 = 524 288 µs；DE = 1；num = 2036；den = 40；ceil = 51；npayload = 8 + 51·8 = 416
  // T = (4·65535 + 17 + 4·416)·2^11·64 = 263 821·131 072 = 34 579 546 112 µs（> 2^32，必須用 64 bit）
  EXPECT_EQ(big, 34579546112ull);
  EXPECT_GT(big, 0xFFFFFFFFull);
  EXPECT_GE(big, loraTimeOnAirUs(c, 254));  // 同一個 ceil 區間可能相等
  EXPECT_GT(big, loraTimeOnAirUs(c, 200));
  EXPECT_EQ(loraAirtimeTicks(c, 255, 0), 0u);
  EXPECT_EQ(loraAirtimeTicks(cfg(7, 125000), 0, 10000), 0u);
  EXPECT_EQ(loraAirtimeTicks(cfg(7, 125000), 256, 10000), 0u);
}

// ---------------- 第 3 項：RfDriverCore TX 完成 ----------------

TEST(RfAirtime, Core_Busy40TicksAt255B_AndThroughput) {
  // 預設參數、rg100Hz（10 ms）：255 B = 399.616 ms → ceil(39.96) = 40 tick／幀
  // 連續 10 s（1000 tick）：第 0、40、…、960 tick 各送一幀 = 25 幀 = 2.5 幀/s、25·255/10 = 637.5 B/s
  RfDriverSimDevice bus;
  NullSink sink;
  InfiniteTxQueue txq(255);
  RfDriverCore drv(bus, sink, txq);
  RfParams p;
  p.txTimeout_s = 1.0;
  bus.configureFrom(p);
  drv.configure(p);
  double t = 0.0;
  toReady(drv, bus, t, 0.01);
  const uint32_t written0 = bus.framesWritten();
  ASSERT_EQ(written0, 0u);

  // 第一幀：恰好 40 tick 後才完成
  bus.advance(); drv.tick(t); t += 0.01;  // tick 0
  ASSERT_EQ(drv.lastTxAirtimeUs(), 399616u);
  ASSERT_EQ(drv.lastTxAirtimeTicks(), 40u);
  for (uint32_t k = 1; k < 40; ++k) {
    bus.advance(); drv.tick(t); t += 0.01;
    ASSERT_TRUE(drv.txInFlight()) << k;
    ASSERT_EQ(drv.sentFrames(), 0u) << k;
    ASSERT_FALSE(sink.comReady) << k;
    ASSERT_TRUE(bus.txBusy()) << k;
  }
  bus.advance(); drv.tick(t); t += 0.01;  // tick 40：完成並送第 2 幀
  EXPECT_EQ(drv.sentFrames(), 1u);
  EXPECT_EQ(bus.framesWritten(), 2u);

  for (uint32_t k = 41; k < 1000; ++k) {
    bus.advance(); drv.tick(t); t += 0.01;
  }
  // 10 s 內交給無線電的幀
  EXPECT_EQ(bus.framesWritten(), 25u);
  EXPECT_EQ(bus.framesWritten() * 255u, 6375u);  // 6375 B / 10 s = 637.5 B/s
  EXPECT_EQ(drv.sentFrames(), 24u);              // 第 25 幀在 tick 1000 才落地
  bus.advance(); drv.tick(t);
  EXPECT_EQ(drv.sentFrames(), 25u);
  EXPECT_EQ(bus.framesRejectedBusy(), 0u);
  EXPECT_EQ(drv.txDroppedCount(), 0u);
  EXPECT_EQ(drv.txTimeoutCount(), 0u);
}

TEST(RfAirtime, Core_TickPeriodConfigurable) {
  // 同一包 255 B（399.616 ms）：rg50Hz（20 ms）→ ceil(19.98) = 20 tick；rg1kHz（1 ms）→ 400 tick
  for (const uint32_t period : {20000u, 1000u}) {
    RfDriverSimDevice bus;
    NullSink sink;
    InfiniteTxQueue txq(255);
    txq.limit(1);
    RfDriverCore drv(bus, sink, txq);
    RfParams p;
    p.tickPeriod_us = period;
    bus.configureFrom(p);
    drv.configure(p);
    const double dt = period * 1e-6;
    double t = 0.0;
    toReady(drv, bus, t, dt);
    bus.advance(); drv.tick(t); t += dt;
    const uint32_t n = (period == 20000u) ? 20u : 400u;
    EXPECT_EQ(drv.lastTxAirtimeTicks(), n);
    for (uint32_t k = 1; k < n; ++k) {
      bus.advance(); drv.tick(t); t += dt;
    }
    EXPECT_EQ(drv.sentFrames(), 0u);
    bus.advance(); drv.tick(t);
    EXPECT_EQ(drv.sentFrames(), 1u);
  }
}

TEST(RfAirtime, Core_SlowConfigNoFalseTimeout) {
  // SF12／125k：255 B 要 9.019 s，遠超 txTimeout_s = 1 s；逾時從空中時間之後算，不能誤判
  RfDriverSimDevice bus;
  NullSink sink;
  InfiniteTxQueue txq(255);
  txq.limit(1);
  RfDriverCore drv(bus, sink, txq);
  RfParams p;
  p.spreadingFactor = 12;
  p.txTimeout_s = 1.0;
  bus.configureFrom(p);
  drv.configure(p);
  double t = 0.0;
  toReady(drv, bus, t, 0.01);
  bus.advance(); drv.tick(t); t += 0.01;
  EXPECT_EQ(drv.lastTxAirtimeTicks(), 902u);  // ceil(901.9392)
  for (uint32_t k = 1; k <= 902; ++k) {
    bus.advance(); drv.tick(t); t += 0.01;
  }
  EXPECT_EQ(drv.sentFrames(), 1u);
  EXPECT_EQ(drv.txTimeoutCount(), 0u);
}

TEST(RfAirtime, Core_TxDoneIrqPath) {
  // 有 TxDone IRQ 來源：不看 tick 數，等 IRQ
  IrqBus bus;
  bus.setAirtimeEnabled(false);  // 真機替身：佔線由 IRQ 決定
  NullSink sink;
  InfiniteTxQueue txq(255);
  txq.limit(2);
  RfDriverCore drv(bus, sink, txq);
  drv.setTxDoneSource(&bus);
  RfParams p;
  p.txTimeout_s = 0.5;
  drv.configure(p);
  double t = 0.0;
  toReady(drv, bus, t, 0.01);
  bus.advance(); drv.tick(t); t += 0.01;  // 送出
  for (int k = 0; k < 60; ++k) {          // 超過模型的 40 tick，但 IRQ 還沒來 → 還在送
    bus.advance(); drv.tick(t); t += 0.01;
  }
  EXPECT_TRUE(drv.txInFlight());
  EXPECT_EQ(drv.sentFrames(), 0u);
  bus.irq = true;
  bus.advance(); drv.tick(t); t += 0.01;
  EXPECT_EQ(drv.sentFrames(), 1u);
  EXPECT_TRUE(drv.txInFlight());  // 同一 tick 送第 2 包
  // 第 2 包 IRQ 一直不來：超過 399.616 ms + 0.5 s → TX_TIMEOUT
  for (int k = 0; k < 85; ++k) {
    bus.advance(); drv.tick(t); t += 0.01;
  }
  EXPECT_EQ(drv.txTimeoutCount(), 0u);
  for (int k = 0; k < 10; ++k) {
    bus.advance(); drv.tick(t); t += 0.01;
  }
  EXPECT_EQ(drv.txTimeoutCount(), 1u);
  EXPECT_TRUE(sink.has(RfEvent::TX_TIMEOUT));
  EXPECT_EQ(bus.abortCalls, 1u);  // 逾時時通知傳輸層放棄這一幀
}

TEST(RfAirtime, Core_OversizeFrameRejected) {
  // SX1278 單包上限 255 B：256 B 不交給硬體，記 TX_FAIL
  RfDriverSimDevice bus;
  NullSink sink;
  InfiniteTxQueue txq(256);
  txq.limit(1);
  RfDriverCore drv(bus, sink, txq);
  RfParams p;
  bus.configureFrom(p);
  drv.configure(p);
  double t = 0.0;
  toReady(drv, bus, t, 0.01);
  bus.advance(); drv.tick(t);
  EXPECT_EQ(bus.framesWritten(), 0u);
  EXPECT_EQ(bus.framesRejectedLength(), 0u);  // 根本沒呼叫 writeFrame
  EXPECT_EQ(drv.txDroppedCount(), 1u);
  EXPECT_TRUE(sink.has(RfEvent::TX_FAIL));
}

// ---------------- 第 3 項（Sim）：latencyTicks ----------------

TEST(RfAirtime, Sim_UplinkLatencyTicks) {
  RfDriverSimDevice dev;
  dev.setLatencyTicks(3);
  const uint8_t up[2] = {0xA5, 0x5A};
  dev.pushUplink(up, 2);
  uint8_t buf[8] = {};
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(dev.readBytes(buf, sizeof(buf)), 0) << k;
    dev.advance();
  }
  EXPECT_EQ(dev.readBytes(buf, sizeof(buf)), 2);
  EXPECT_EQ(buf[0], 0xA5);
  EXPECT_EQ(buf[1], 0x5A);
  // 0 延遲＝同一 tick 就讀得到（舊行為）
  dev.setLatencyTicks(0);
  dev.pushUplink(up, 1);
  EXPECT_EQ(dev.readBytes(buf, sizeof(buf)), 1);
  // 兩段不同延遲：先推的（延遲 5）沒到，後推的（延遲 1）也不能插隊
  dev.setLatencyTicks(5);
  dev.pushUplink(up, 2);
  dev.setLatencyTicks(1);
  dev.pushUplink(up, 2);
  dev.advance();
  EXPECT_EQ(dev.readBytes(buf, sizeof(buf)), 0);
  for (int k = 0; k < 4; ++k) dev.advance();
  EXPECT_EQ(dev.readBytes(buf, sizeof(buf)), 4);
  // 超過 16 段：併段，不掉位元組
  dev.setLatencyTicks(2);
  for (int k = 0; k < 40; ++k) dev.pushUplink(up, 1);
  EXPECT_EQ(dev.readBytes(buf, sizeof(buf)), 0);
  dev.advance();
  dev.advance();
  uint8_t big[64] = {};
  EXPECT_EQ(dev.readBytes(big, sizeof(big)), 40);
}

TEST(RfAirtime, Sim_ConfigureFromRfParamsMatchesCore) {
  RfParams p;
  p.spreadingFactor = 9;
  p.bandwidth = LoraBw::BW_250K;
  p.tickPeriod_us = 5000;
  RfDriverSimDevice dev;
  dev.configureFrom(p);
  EXPECT_EQ(dev.tickPeriodUs(), 5000u);
  // SF9/250k：Tsym 2.048 ms，npayload 293 → 305.25·2.048 = 625.152 ms → ceil(125.03) = 126 tick
  EXPECT_EQ(dev.airtimeTicks(255), 126u);
  EXPECT_EQ(loraAirtimeTicks(loraConfigFromRfParams(p), 255, p.tickPeriod_us), 126u);
}
