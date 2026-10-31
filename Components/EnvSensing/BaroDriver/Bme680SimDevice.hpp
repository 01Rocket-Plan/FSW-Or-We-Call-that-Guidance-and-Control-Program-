// Bme680SimDevice：BME680 暫存器層模擬（給 BaroDriverSim 與 host 測試）
// 用典型校正係數，把「真值」反推成原始 ADC，讓真正的驅動核心走完整路徑。
#ifndef ENVSENSING_BME680SIMDEVICE_HPP
#define ENVSENSING_BME680SIMDEVICE_HPP
#include "Components/EnvSensing/BaroDriver/Bme680Core.hpp"
#include <cstdint>

namespace EnvSensing {

enum class Bme680SimFault : uint8_t { NONE = 0, NACK = 1, STUCK = 2, OUT_OF_RANGE = 3, BAD_CHIP_ID = 4, NO_DATA = 5 };

class Bme680SimDevice : public IBme680Bus {
 public:
  Bme680SimDevice();
  // 每個 tick 在驅動 tick 之前呼叫
  void advance();
  void setTruth(double press_Pa, double temp_C, double hum_pct, double gas_ohm);
  void setAltitude(double alt_m);  // 以 101325 Pa、15 °C 標準大氣換算氣壓
  void setFault(Bme680SimFault f) { m_fault = f; }
  bool writeReg(uint8_t reg, uint8_t val) override;
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) override;
  static void typicalCoeff(uint8_t* coeff42);

 private:
  void complete();
  uint8_t m_reg[256];
  Bme680Calib m_cal{};
  Bme680SimFault m_fault = Bme680SimFault::NONE;
  double m_p = 101325.0, m_t = 25.0, m_h = 45.0, m_g = 50000.0;
  int m_pending = -1;  // 轉換剩餘 tick
  bool m_pendingGas = false;
  uint32_t m_rng = 12345u;
};

}  // namespace EnvSensing
#endif
