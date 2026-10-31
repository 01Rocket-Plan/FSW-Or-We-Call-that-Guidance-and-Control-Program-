#include "Components/EnvSensing/BaroDriver/Bme680SimDevice.hpp"
#include "Components/EnvSensing/BaroDriver/Bme680Regs.hpp"
#include <cmath>
#include <cstring>

namespace EnvSensing {
using namespace Bme680Reg;

void Bme680SimDevice::typicalCoeff(uint8_t* a) {
  std::memset(a, 0, 42);
  auto put16 = [&](int lsbIdx, int msbIdx, int v) {
    a[lsbIdx] = static_cast<uint8_t>(v & 0xFF);
    a[msbIdx] = static_cast<uint8_t>((v >> 8) & 0xFF);
  };
  // 典型 BME680 校正值（實機讀值量級）
  put16(31, 32, 26111);   // t1
  put16(0, 1, 26256);     // t2
  a[2] = 3;               // t3
  put16(4, 5, 36477);     // p1
  put16(6, 7, -10423);    // p2
  a[8] = 88;              // p3
  put16(10, 11, 8136);    // p4
  put16(12, 13, -68);     // p5
  a[15] = 30;             // p6
  a[14] = 50;             // p7
  put16(18, 19, -2357);   // p8
  put16(20, 21, -3140);   // p9
  a[22] = 30;             // p10
  int h1 = 794, h2 = 1019;
  a[25] = static_cast<uint8_t>(h1 >> 4);
  a[24] = static_cast<uint8_t>((h1 & 0x0F) | ((h2 & 0x0F) << 4));
  a[23] = static_cast<uint8_t>(h2 >> 4);
  a[26] = 0;  a[27] = 45;  a[28] = 20;  a[29] = 120;  a[30] = static_cast<uint8_t>(-100);
  a[35] = static_cast<uint8_t>(-30);   // gh1
  put16(33, 34, -5969);   // gh2
  a[36] = 18;             // gh3
  a[37] = 41;             // res_heat_val
  a[39] = 0x10;           // res_heat_range = 1
  a[41] = 0;              // range_sw_err
}

Bme680SimDevice::Bme680SimDevice() {
  std::memset(m_reg, 0, sizeof(m_reg));
  uint8_t a[42];
  typicalCoeff(a);
  std::memcpy(&m_reg[COEFF1], a, COEFF1_LEN);
  std::memcpy(&m_reg[COEFF2], a + COEFF1_LEN, COEFF2_LEN);
  std::memcpy(&m_reg[COEFF3], a + COEFF1_LEN + COEFF2_LEN, COEFF3_LEN);
  Bme680Math::parseCalib(a, m_cal);
  m_reg[CHIP_ID] = CHIP_ID_VAL;
}

void Bme680SimDevice::setTruth(double p, double t, double h, double g) { m_p = p; m_t = t; m_h = h; m_g = g; }

void Bme680SimDevice::setAltitude(double alt) { m_p = 101325.0 * std::pow(1.0 - alt / 44330.0, 5.255); }

bool Bme680SimDevice::writeReg(uint8_t reg, uint8_t val) {
  if (m_fault == Bme680SimFault::NACK) return false;
  if (reg == RESET && val == RESET_CMD) {
    m_pending = -1;
    m_reg[FIELD0] = 0;
    return true;
  }
  m_reg[reg] = val;
  if (reg == CTRL_MEAS && (val & 0x03) == MODE_FORCED) {
    m_pendingGas = (m_reg[CTRL_GAS_1] & RUN_GAS) != 0;
    // 溫壓 x1：約 9 ms，1 tick 內完成；含氣體（30 ms 加熱）約 3–4 tick
    m_pending = m_pendingGas ? 4 : 1;
    m_reg[FIELD0] = STATUS_MEASURING | (m_pendingGas ? STATUS_GAS_MEAS : 0);
  }
  return true;
}

bool Bme680SimDevice::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  if (m_fault == Bme680SimFault::NACK) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = m_reg[static_cast<uint8_t>(reg + i)];
  if (m_fault == Bme680SimFault::BAD_CHIP_ID && reg <= CHIP_ID && CHIP_ID < reg + len) buf[CHIP_ID - reg] = 0x00;
  return true;
}

void Bme680SimDevice::advance() {
  if (m_pending > 0 && --m_pending == 0) {
    if (m_fault != Bme680SimFault::NO_DATA) complete();
    else m_pending = 1;
  }
}

template <typename F>
static uint32_t invert(F f, double target, uint32_t lo, uint32_t hi, bool increasing) {
  for (int i = 0; i < 40 && lo < hi; i++) {
    uint32_t mid = lo + (hi - lo) / 2;
    double v = f(mid);
    if ((v < target) == increasing) lo = mid + 1; else hi = mid;
  }
  return lo;
}

void Bme680SimDevice::complete() {
  if (m_fault != Bme680SimFault::STUCK) {
    m_rng = m_rng * 1103515245u + 12345u;
    double nP = ((m_rng >> 16) % 401) / 100.0 - 2.0;    // ±2 Pa
    double nT = ((m_rng >> 8) % 41) / 1000.0 - 0.02;    // ±0.02 °C
    double p = (m_fault == Bme680SimFault::OUT_OF_RANGE) ? 500.0 : m_p + nP;
    double t = m_t + nT;
    double tf = 0;
    uint32_t tAdc = invert([&](uint32_t a) { double x; return Bme680Math::compTemp(m_cal, a, x); }, t, 0, 0xFFFFF, true);
    Bme680Math::compTemp(m_cal, tAdc, tf);
    uint32_t pAdc = invert([&](uint32_t a) { return Bme680Math::compPress(m_cal, a, tf); }, p, 0, 0xFFFFF, false);
    m_reg[0x1F] = static_cast<uint8_t>(pAdc >> 12);
    m_reg[0x20] = static_cast<uint8_t>(pAdc >> 4);
    m_reg[0x21] = static_cast<uint8_t>((pAdc & 0x0F) << 4);
    m_reg[0x22] = static_cast<uint8_t>(tAdc >> 12);
    m_reg[0x23] = static_cast<uint8_t>(tAdc >> 4);
    m_reg[0x24] = static_cast<uint8_t>((tAdc & 0x0F) << 4);
    if (m_pendingGas) {
      uint32_t hAdc = invert([&](uint32_t a) { return Bme680Math::compHum(m_cal, static_cast<uint16_t>(a), tf); }, m_h, 0, 0xFFFF, true);
      // 選一個能涵蓋目標電阻的量程（實機由晶片自動選）
      uint8_t range = 0;
      for (uint8_t rr = 0; rr < 16; rr++) {
        if (Bme680Math::compGas(m_cal, 1023, rr) <= m_g && m_g <= Bme680Math::compGas(m_cal, 0, rr)) { range = rr; break; }
      }
      uint32_t gAdc = invert([&](uint32_t a) { return Bme680Math::compGas(m_cal, static_cast<uint16_t>(a), range); }, m_g, 0, 1023, false);
      m_reg[0x25] = static_cast<uint8_t>(hAdc >> 8);
      m_reg[0x26] = static_cast<uint8_t>(hAdc);
      m_reg[0x2A] = static_cast<uint8_t>(gAdc >> 2);
      m_reg[0x2B] = static_cast<uint8_t>(((gAdc & 0x03) << 6) | GAS_VALID | HEAT_STAB | range);
    } else {
      m_reg[0x25] = 0x80; m_reg[0x26] = 0x00;  // 濕度跳過
      m_reg[0x2B] = 0x00;
    }
  }
  m_reg[FIELD0] = STATUS_NEW_DATA;
}

}  // namespace EnvSensing
