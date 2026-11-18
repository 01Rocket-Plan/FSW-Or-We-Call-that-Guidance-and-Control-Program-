// LoraTimeOnAirTest：LoRa 空中時間＝Semtech SX1276/77/78/79 datasheet Rev. 4 §4.1.1.7 公式值；
// RfDriverSimDevice 佔線模型（issue #60 第 5 項：給定參數，255 B 一幀的佔線時間＝公式值）。
//
// 期望值都是把公式手算一遍（寫在每個測試的註解裡），不是從程式輸出抄回來的。
#include "Components/Wireless/RfDriverSim/LoraTimeOnAir.hpp"
#include "Components/Wireless/RfDriverSim/RfDriverSimDevice.hpp"
#include <gtest/gtest.h>

using namespace Components;

namespace {
constexpr double kTol_us = 1e-6;
}

TEST(LoraTimeOnAir, ResetDefaultsMatchDatasheetRegisterTable) {
  const LoraModemConfig c{};
  EXPECT_EQ(c.sf, 7u);
  EXPECT_EQ(c.bw_Hz, 125000u);
  EXPECT_EQ(c.cr, 1u);
  EXPECT_EQ(c.preambleLen, 8u);
  EXPECT_TRUE(c.explicitHeader);
  EXPECT_FALSE(c.crcOn);
  EXPECT_TRUE(loraConfigValid(c));
  EXPECT_EQ(loraLowDataRateOptimize(c), 0u);  // Tsym 1.024 ms < 16 ms
}

TEST(LoraTimeOnAir, SymbolTime) {
  LoraModemConfig c{};
  EXPECT_NEAR(loraSymbolTime_us(c), 1024.0, kTol_us);  // 2^7 / 125 kHz
  c.sf = 12;
  EXPECT_NEAR(loraSymbolTime_us(c), 32768.0, kTol_us);  // 2^12 / 125 kHz
  c.bw_Hz = 500000;
  EXPECT_NEAR(loraSymbolTime_us(c), 8192.0, kTol_us);  // 2^12 / 500 kHz
}

TEST(LoraTimeOnAir, Sf7Bw125Cr45_255B_CrcOff) {
  // num = 8·255 − 28 + 28 + 0 − 0 = 2040；den = 28；ceil(72.86) = 73；npayload = 8 + 73·5 = 373
  // T = (8 + 4.25 + 373) · 1024 µs = 394 496 µs
  const LoraModemConfig c{};
  EXPECT_EQ(loraPayloadSymbols(c, 255), 373u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 255), 394496.0, kTol_us);
}

TEST(LoraTimeOnAir, Sf7Bw125Cr45_255B_CrcOn) {
  // num = 2040 + 16 = 2056；ceil(2056/28 = 73.43) = 74；npayload = 8 + 74·5 = 378
  // T = (12.25 + 378) · 1024 = 399 616 µs
  LoraModemConfig c{};
  c.crcOn = true;
  EXPECT_EQ(loraPayloadSymbols(c, 255), 378u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 255), 399616.0, kTol_us);
}

TEST(LoraTimeOnAir, Sf12Bw125_LdroAuto_10B_CrcOn) {
  // Tsym = 32.768 ms > 16 ms → DE = 1（§4.1.1.6）
  // num = 80 − 48 + 28 + 16 = 76；den = 4·(12 − 2) = 40；ceil(1.9) = 2；npayload = 8 + 2·5 = 18
  // T = (12.25 + 18) · 32768 = 991 232 µs
  LoraModemConfig c{};
  c.sf = 12;
  c.crcOn = true;
  EXPECT_EQ(loraLowDataRateOptimize(c), 1u);
  EXPECT_EQ(loraPayloadSymbols(c, 10), 18u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 10), 991232.0, kTol_us);
  // 強制關 LDRO：den = 48；ceil(76/48) = 2 → 同樣 18 符號（這組剛好不變）
  c.ldro = LoraLdro::OFF;
  EXPECT_EQ(loraPayloadSymbols(c, 10), 18u);
}

TEST(LoraTimeOnAir, Sf12Bw125_64B_CrcOn_LdroMatters) {
  // num = 8·64 − 48 + 28 + 16 = 508
  // LDRO 自動（開）：den = 40；ceil(12.7) = 13；npayload = 8 + 13·5 = 73；T = (12.25 + 73)·32768 = 2 793 472 µs
  // LDRO 關：den = 48；ceil(10.58) = 11；npayload = 8 + 11·5 = 63
  LoraModemConfig c{};
  c.sf = 12;
  c.crcOn = true;
  EXPECT_EQ(loraPayloadSymbols(c, 64), 73u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 64), 2793472.0, kTol_us);
  c.ldro = LoraLdro::OFF;
  EXPECT_EQ(loraPayloadSymbols(c, 64), 63u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 64), (12.25 + 63.0) * 32768.0, kTol_us);
}

TEST(LoraTimeOnAir, Sf7Bw125_13B_CrcOn_HeaderMatters) {
  // 顯式：num = 104 − 28 + 28 + 16 = 120；ceil(120/28 = 4.29) = 5；npayload = 33；T = (12.25 + 33)·1024 = 46 336 µs
  // 隱式：num = 120 − 20 = 100；ceil(3.57) = 4；npayload = 28；T = (12.25 + 28)·1024 = 41 216 µs
  LoraModemConfig c{};
  c.crcOn = true;
  EXPECT_EQ(loraPayloadSymbols(c, 13), 33u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 13), 46336.0, kTol_us);
  c.explicitHeader = false;
  EXPECT_EQ(loraPayloadSymbols(c, 13), 28u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 13), 41216.0, kTol_us);
}

TEST(LoraTimeOnAir, Sf12Bw125_13B_CrcOn) {
  // num = 104 − 48 + 28 + 16 = 100；DE = 1 → den = 40；ceil(2.5) = 3；npayload = 23；T = (12.25 + 23)·32768 = 1 155 072 µs
  LoraModemConfig c{};
  c.sf = 12;
  c.crcOn = true;
  EXPECT_EQ(loraPayloadSymbols(c, 13), 23u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 13), 1155072.0, kTol_us);
}

TEST(LoraTimeOnAir, CodingRateAndImplicitHeader) {
  // CR 4/8：num = 2040，ceil = 73，npayload = 8 + 73·8 = 592；T = (12.25 + 592)·1024 = 618 752 µs
  LoraModemConfig c{};
  c.cr = 4;
  EXPECT_EQ(loraPayloadSymbols(c, 255), 592u);
  EXPECT_NEAR(loraTimeOnAir_us(c, 255), 618752.0, kTol_us);
  // 隱式標頭 4/5：num = 2040 − 20 = 2020；ceil(72.14) = 73 → 373（同顯式；本例剛好不變）
  c.cr = 1;
  c.explicitHeader = false;
  EXPECT_EQ(loraPayloadSymbols(c, 255), 373u);
  // 1 B、顯式、CRC 關：num = 8 − 28 + 28 = 8；ceil(8/28) = 1 → 8 + 5 = 13
  c.explicitHeader = true;
  EXPECT_EQ(loraPayloadSymbols(c, 1), 13u);
}

TEST(LoraTimeOnAir, RejectsInvalid) {
  LoraModemConfig c{};
  EXPECT_EQ(loraTimeOnAir_us(c, 0), 0.0);
  EXPECT_EQ(loraTimeOnAir_us(c, 256), 0.0);
  c.sf = 6;  // SF6 + 顯式標頭不允許（§4.1.1.2）
  EXPECT_FALSE(loraConfigValid(c));
  c.explicitHeader = false;
  EXPECT_TRUE(loraConfigValid(c));
  c = LoraModemConfig{};
  c.bw_Hz = 100000;  // 不在 RegModemConfig1.Bw 表內
  EXPECT_FALSE(loraConfigValid(c));
  c = LoraModemConfig{};
  c.cr = 5;
  EXPECT_FALSE(loraConfigValid(c));
  c = LoraModemConfig{};
  c.sf = 13;
  EXPECT_FALSE(loraConfigValid(c));
}

TEST(RfDriverSimDevice, AirtimeOccupiesChannel255B) {
  // 預設 10 ms tick：394.496 ms → ceil(39.4496) = 40 tick
  RfDriverSimDevice dev;
  ASSERT_TRUE(dev.init());
  EXPECT_EQ(dev.airtimeTicks(255), 40u);
  uint8_t buf[255] = {};
  EXPECT_EQ(dev.writeFrame(buf, 255), 255);
  EXPECT_EQ(dev.lastAirtimeUs(), 394496u);
  EXPECT_TRUE(dev.ready());  // ready() 不表示佔線（見 RfDriverSimDevice.hpp）
  for (uint32_t i = 1; i < 40; ++i) {
    dev.advance();
    EXPECT_TRUE(dev.txBusy()) << "tick " << i;
    EXPECT_EQ(dev.writeFrame(buf, 255), 0) << "tick " << i;
  }
  EXPECT_EQ(dev.framesRejectedBusy(), 39u);
  dev.advance();  // 第 40 tick：空中時間結束
  EXPECT_FALSE(dev.txBusy());
  EXPECT_EQ(dev.writeFrame(buf, 255), 255);
  EXPECT_EQ(dev.framesWritten(), 2u);
}

TEST(RfDriverSimDevice, TickPeriodAndParamsChangeAirtime) {
  RfDriverSimDevice dev;
  dev.setTickPeriodUs(1000);  // 1 kHz 只是換算用，不代表要掛 1 kHz
  EXPECT_EQ(dev.airtimeTicks(255), 395u);  // ceil(394.496)
  LoraModemConfig c{};
  c.sf = 12;
  c.crcOn = true;
  dev.setModem(c);
  dev.setTickPeriodUs(10000);
  EXPECT_EQ(dev.airtimeTicks(10), 100u);  // ceil(991.232 / 10) = 100
}

TEST(RfDriverSimDevice, RejectsOversizeAndCanDisableModel) {
  RfDriverSimDevice dev;
  uint8_t buf[300] = {};
  EXPECT_EQ(dev.writeFrame(buf, 256), 0);  // SX1278 PL 上限 255
  EXPECT_EQ(dev.framesRejectedLength(), 1u);
  dev.setAirtimeEnabled(false);
  EXPECT_EQ(dev.writeFrame(buf, 4), 4);
  EXPECT_EQ(dev.writeFrame(buf, 4), 4);  // 關掉模型：同一 tick 也能再送（舊行為）
  EXPECT_FALSE(dev.txBusy());
}
