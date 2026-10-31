#include "Components/EnvSensing/BaroDriver/Bme680Core.hpp"
#include "Components/EnvSensing/BaroDriver/Bme680Regs.hpp"
#include <cmath>
#include <limits>

namespace Components {
using namespace Bme680Reg;

namespace Bme680Math {

static inline int16_t s16(uint8_t msb, uint8_t lsb) { return static_cast<int16_t>((msb << 8) | lsb); }
static inline uint16_t u16(uint8_t msb, uint8_t lsb) { return static_cast<uint16_t>((msb << 8) | lsb); }

bool parseCalib(const uint8_t* a, Bme680Calib& c) {
  // a = COEFF1(23) + COEFF2(14) + COEFF3(5)，索引同 Bosch bme68x_defs.h
  c.t1 = u16(a[32], a[31]);
  c.t2 = s16(a[1], a[0]);
  c.t3 = static_cast<int8_t>(a[2]);
  c.p1 = u16(a[5], a[4]);
  c.p2 = s16(a[7], a[6]);
  c.p3 = static_cast<int8_t>(a[8]);
  c.p4 = s16(a[11], a[10]);
  c.p5 = s16(a[13], a[12]);
  c.p6 = static_cast<int8_t>(a[15]);
  c.p7 = static_cast<int8_t>(a[14]);
  c.p8 = s16(a[19], a[18]);
  c.p9 = s16(a[21], a[20]);
  c.p10 = a[22];
  c.h1 = static_cast<uint16_t>((a[25] << 4) | (a[24] & 0x0F));
  c.h2 = static_cast<uint16_t>((a[23] << 4) | (a[24] >> 4));
  c.h3 = static_cast<int8_t>(a[26]);
  c.h4 = static_cast<int8_t>(a[27]);
  c.h5 = static_cast<int8_t>(a[28]);
  c.h6 = a[29];
  c.h7 = static_cast<int8_t>(a[30]);
  c.gh1 = static_cast<int8_t>(a[35]);
  c.gh2 = s16(a[34], a[33]);
  c.gh3 = static_cast<int8_t>(a[36]);
  c.resHeatVal = static_cast<int8_t>(a[37]);
  c.resHeatRange = (a[39] & 0x30) >> 4;
  c.rangeSwErr = (static_cast<int8_t>(a[41]) & 0xF0) / 16;
  // 全 0 或全 FF 視為讀壞
  return c.t1 != 0 && c.t1 != 0xFFFF && c.p1 != 0 && c.p1 != 0xFFFF;
}

double compTemp(const Bme680Calib& c, uint32_t adc, double& tFine) {
  double v1 = ((adc / 16384.0) - (c.t1 / 1024.0)) * c.t2;
  double d = (adc / 131072.0) - (c.t1 / 8192.0);
  double v2 = d * d * (c.t3 * 16.0);
  tFine = v1 + v2;
  return tFine / 5120.0;
}

double compPress(const Bme680Calib& c, uint32_t adc, double tFine) {
  double v1 = (tFine / 2.0) - 64000.0;
  double v2 = v1 * v1 * (c.p6 / 131072.0);
  v2 = v2 + (v1 * c.p5 * 2.0);
  v2 = (v2 / 4.0) + (c.p4 * 65536.0);
  v1 = (((c.p3 * v1 * v1) / 16384.0) + (c.p2 * v1)) / 524288.0;
  v1 = (1.0 + (v1 / 32768.0)) * c.p1;
  double p = 1048576.0 - adc;
  if (v1 == 0.0) return 0.0;
  p = ((p - (v2 / 4096.0)) * 6250.0) / v1;
  v1 = (c.p9 * p * p) / 2147483648.0;
  v2 = p * (c.p8 / 32768.0);
  double v3 = (p / 256.0) * (p / 256.0) * (p / 256.0) * (c.p10 / 131072.0);
  return p + (v1 + v2 + v3 + (c.p7 * 128.0)) / 16.0;
}

double compHum(const Bme680Calib& c, uint16_t adc, double tFine) {
  double tc = tFine / 5120.0;
  double v1 = adc - ((c.h1 * 16.0) + ((c.h3 / 2.0) * tc));
  double v2 = v1 * ((c.h2 / 262144.0) * (1.0 + ((c.h4 / 16384.0) * tc) + ((c.h5 / 1048576.0) * tc * tc)));
  double v3 = c.h6 / 16384.0;
  double v4 = c.h7 / 2097152.0;
  double h = v2 + ((v3 + (v4 * tc)) * v2 * v2);
  if (h > 100.0) h = 100.0;
  if (h < 0.0) h = 0.0;
  return h;
}

double compGas(const Bme680Calib& c, uint16_t adc, uint8_t range) {
  static const double k1[16] = {0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0};
  static const double k2[16] = {0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  range &= 0x0F;
  double v1 = 1340.0 + (5.0 * c.rangeSwErr);
  double v2 = v1 * (1.0 + k1[range] / 100.0);
  double v3 = 1.0 + (k2[range] / 100.0);
  return 1.0 / (v3 * 0.000000125 * static_cast<double>(1u << range) * (((adc - 512.0) / v2) + 1.0));
}

uint8_t heaterRes(const Bme680Calib& c, double target_C, double amb_C) {
  if (target_C > 400.0) target_C = 400.0;
  double v1 = (c.gh1 / 16.0) + 49.0;
  double v2 = ((c.gh2 / 32768.0) * 0.0005) + 0.00235;
  double v3 = c.gh3 / 1024.0;
  double v4 = v1 * (1.0 + (v2 * target_C));
  double v5 = v4 + (v3 * amb_C);
  double r = 3.4 * ((v5 * (4.0 / (4.0 + c.resHeatRange)) * (1.0 / (1.0 + (c.resHeatVal * 0.002)))) - 25.0);
  if (r < 0.0) r = 0.0;
  if (r > 255.0) r = 255.0;
  return static_cast<uint8_t>(r);
}

uint8_t gasWaitCode(uint16_t ms) {
  if (ms >= 0xFC0) return 0xFF;
  uint8_t factor = 0;
  while (ms > 0x3F) { ms /= 4; factor++; }
  return static_cast<uint8_t>(ms + factor * 64);
}

double altFromPress(double p, double p0) {
  if (p0 <= 0.0 || p <= 0.0) return std::numeric_limits<double>::quiet_NaN();
  return 44330.0 * (1.0 - std::pow(p / p0, 0.190294957));
}

}  // namespace Bme680Math

Bme680Core::Bme680Core(IBme680Bus& bus, IBme680Sink& sink) : m_bus(bus), m_sink(sink) {}

void Bme680Core::setConfig(const Bme680Config& cfg) {
  m_cfg = cfg;
  if (cfg.p0_Pa > 0.0f) m_p0 = cfg.p0_Pa;
}

void Bme680Core::setState(DrvState s) {
  if (s == m_state) return;
  DrvState from = m_state;
  m_state = s;
  m_sink.onStateChange(from, s);
}

void Bme680Core::fault(BaroFault code, uint32_t detail) {
  m_sink.onFault(code, detail);
  m_measuring = false;
  m_faultTicks = 0;
  setState(DrvState::FAULT);
}

bool Bme680Core::busErr() {
  m_totalErrors++;
  if (++m_consecErr >= m_cfg.errMax) {
    fault(BaroFault::BUS_ERROR, m_consecErr);
    return true;
  }
  return false;
}

void Bme680Core::tick(double t_s) {
  switch (m_state) {
    case DrvState::OFF:
      m_initStep = 0;
      m_consecErr = 0;
      setState(DrvState::INIT);
      tickInit();
      break;
    case DrvState::INIT:
      tickInit();
      break;
    case DrvState::SELFTEST:
    case DrvState::READY:
      tickMeasure(t_s);
      break;
    case DrvState::FAULT:
      if (m_retries < m_cfg.maxRetries && ++m_faultTicks >= m_cfg.faultRetryTicks) {
        m_retries++;
        m_initStep = 0;
        m_consecErr = 0;
        setState(DrvState::INIT);
      }
      break;
  }
}

void Bme680Core::tickInit() {
  switch (m_initStep) {
    case 0:  // 軟重置；下一個 tick（10 ms）後重置早已完成（datasheet 2 ms）
      if (!m_bus.writeReg(RESET, RESET_CMD)) { busErr(); return; }
      m_initStep = 1;
      return;
    case 1: {
      uint8_t id = 0;
      if (!m_bus.readRegs(CHIP_ID, &id, 1)) { busErr(); return; }
      if (id != CHIP_ID_VAL) { fault(BaroFault::BAD_CHIP_ID, id); return; }
      m_initStep = 2;
      return;
    }
    case 2: {
      uint8_t a[COEFF1_LEN + COEFF2_LEN + COEFF3_LEN];
      if (!m_bus.readRegs(COEFF1, a, COEFF1_LEN) ||
          !m_bus.readRegs(COEFF2, a + COEFF1_LEN, COEFF2_LEN) ||
          !m_bus.readRegs(COEFF3, a + COEFF1_LEN + COEFF2_LEN, COEFF3_LEN)) { busErr(); return; }
      if (!Bme680Math::parseCalib(a, m_cal)) { fault(BaroFault::SELFTEST_FAIL, 0xCA1); return; }
      m_initStep = 3;
      return;
    }
    case 3: {
      bool ok = m_bus.writeReg(CTRL_HUM, 0x00) &&
                m_bus.writeReg(CONFIG, static_cast<uint8_t>((m_cfg.filter & 0x07) << 2)) &&
                m_bus.writeReg(CTRL_GAS_1, 0x00) &&
                m_bus.writeReg(GAS_WAIT_0, Bme680Math::gasWaitCode(m_cfg.gasWait_ms)) &&
                m_bus.writeReg(RES_HEAT_0, Bme680Math::heaterRes(m_cal, m_cfg.heaterTemp_C, m_lastTemp_C));
      if (!ok) { busErr(); return; }
      m_humOn = false;
      m_gasOn = false;
      m_measuring = false;
      m_consecErr = m_consecRange = m_consecTimeout = 0;
      m_stuckCount = 0;
      m_lastPAdc = m_lastTAdc = 0xFFFFFFFF;
      m_selftestGood = 0;
      m_p0Acc = 0.0;
      m_ticksSinceGas = 0;
      setState(DrvState::SELFTEST);
      return;
    }
    default:
      m_initStep = 0;
  }
}

bool Bme680Core::trigger(bool gas) {
  if (gas) {
    if (!m_humOn && !m_bus.writeReg(CTRL_HUM, 0x01)) return false;
    m_humOn = true;
    if (!m_bus.writeReg(RES_HEAT_0, Bme680Math::heaterRes(m_cal, m_cfg.heaterTemp_C, m_lastTemp_C))) return false;
    if (!m_gasOn && !m_bus.writeReg(CTRL_GAS_1, RUN_GAS)) return false;
    m_gasOn = true;
  } else {
    if (m_humOn && !m_bus.writeReg(CTRL_HUM, 0x00)) return false;
    m_humOn = false;
    if (m_gasOn && !m_bus.writeReg(CTRL_GAS_1, 0x00)) return false;
    m_gasOn = false;
  }
  uint8_t meas = static_cast<uint8_t>(((m_cfg.osrsT & 0x07) << 5) | ((m_cfg.osrsP & 0x07) << 2) | MODE_FORCED);
  return m_bus.writeReg(CTRL_MEAS, meas);
}

void Bme680Core::tickMeasure(double t_s) {
  if (m_ticksSinceGas < 0xFFFF) m_ticksSinceGas++;
  if (!m_measuring) {
    // 觸發 tick：溫壓與讀取交替，約 50 Hz；每 gasPeriodTicks 插一次溫壓＋氣體
    bool gas = m_ticksSinceGas >= m_cfg.gasPeriodTicks;
    if (!trigger(gas)) { busErr(); return; }
    m_consecErr = 0;
    m_measuring = true;
    m_measGas = gas;
    if (gas) m_ticksSinceGas = 0;
    m_waitTicks = 0;
    m_trigT = t_s;  // 樣本時間取觸發時刻（F´ 時間服務）
    return;
  }
  m_waitTicks++;
  uint8_t d[FIELD0_LEN];
  if (!m_bus.readRegs(FIELD0, d, FIELD0_LEN)) { busErr(); return; }
  m_consecErr = 0;
  if (!(d[0] & STATUS_NEW_DATA)) {
    uint8_t lim = m_measGas ? m_cfg.gasTimeoutTicks : m_cfg.measTimeoutTicks;
    if (m_waitTicks >= lim) {
      m_measuring = false;  // 放棄這筆，不輸出假樣本、seq 不加
      m_totalErrors++;
      if (++m_consecTimeout >= m_cfg.errMax) fault(BaroFault::MEAS_TIMEOUT, m_consecTimeout);
    }
    return;
  }
  m_consecTimeout = 0;
  m_measuring = false;
  handleData(d, m_measGas);
}

void Bme680Core::handleData(const uint8_t* d, bool gas) {
  uint32_t pAdc = (static_cast<uint32_t>(d[2]) << 12) | (static_cast<uint32_t>(d[3]) << 4) | (d[4] >> 4);
  uint32_t tAdc = (static_cast<uint32_t>(d[5]) << 12) | (static_cast<uint32_t>(d[6]) << 4) | (d[7] >> 4);
  uint16_t hAdc = static_cast<uint16_t>((d[8] << 8) | d[9]);
  uint16_t gAdc = static_cast<uint16_t>((d[13] << 2) | (d[14] >> 6));
  uint8_t gRange = d[14] & 0x0F;
  bool gasOk = (d[14] & GAS_VALID) && (d[14] & HEAT_STAB);

  // 卡值：原始溫壓 ADC 連續相同
  if (pAdc == m_lastPAdc && tAdc == m_lastTAdc) {
    if (m_cfg.stuckN > 0 && ++m_stuckCount >= m_cfg.stuckN) { fault(BaroFault::STUCK, pAdc); return; }
  } else {
    m_stuckCount = 0;
  }
  m_lastPAdc = pAdc;
  m_lastTAdc = tAdc;

  double tFine = 0.0;
  double tC = Bme680Math::compTemp(m_cal, tAdc, tFine);
  double p = Bme680Math::compPress(m_cal, pAdc, tFine);
  bool inRange = (tC > -40.0 && tC < 85.0 && p > 10000.0 && p < 120000.0 && std::isfinite(p));
  if (!inRange) {
    m_totalErrors++;
    if (++m_consecRange >= m_cfg.errMax) {
      fault(m_state == DrvState::SELFTEST ? BaroFault::SELFTEST_FAIL : BaroFault::OUT_OF_RANGE,
            static_cast<uint32_t>(p));
    }
    return;  // 不輸出不合理樣本
  }
  m_consecRange = 0;
  m_lastTemp_C = tC;

  if (m_state == DrvState::SELFTEST) {
    m_selftestGood++;
    m_p0Acc += p;
  }

  bool valid = (m_state == DrvState::READY);
  BaroOut b{m_trigT, m_baroSeq++, valid, static_cast<float>(p), static_cast<float>(tC),
            static_cast<float>(Bme680Math::altFromPress(p, m_p0))};
  m_sink.onBaro(b);

  if (gas) {
    double h = Bme680Math::compHum(m_cal, hAdc, tFine);
    double g = gasOk ? Bme680Math::compGas(m_cal, gAdc, gRange) : std::numeric_limits<double>::quiet_NaN();
    EnvOut e{m_trigT, m_envSeq++, valid, static_cast<float>(h), static_cast<float>(g)};
    m_sink.onEnv(e);
  }

  if (m_state == DrvState::SELFTEST && m_selftestGood >= m_cfg.selftestN) {
    if (m_stuckCount + 1 >= m_selftestGood) { fault(BaroFault::SELFTEST_FAIL, 0x57); return; }  // 自測期間全部同值
    if (m_cfg.p0_Pa <= 0.0f) m_p0 = static_cast<float>(m_p0Acc / m_selftestGood);
    setState(DrvState::READY);
  }
}

}  // namespace Components
