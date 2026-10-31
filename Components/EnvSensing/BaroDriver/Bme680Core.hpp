// Bme680Core：BME680 驅動核心（與 F´ 無關，可在 host 測試）
// 規則：不 sleep、不讀系統時間；時間由呼叫端（F´ 時間服務）每 tick 傳入。
// 每個 tick 最多做一組短 I2C 交易，全部是非阻塞狀態機。
#ifndef ENVSENSING_BME680CORE_HPP
#define ENVSENSING_BME680CORE_HPP

#include <cstdint>

namespace Components {

// 與 RocketTypes.DrvState 順序一致
enum class DrvState : uint8_t { OFF = 0, INIT = 1, SELFTEST = 2, READY = 3, FAULT = 4 };

// 故障碼（事件 Fault 的 code）
enum class BaroFault : uint8_t {
  BUS_ERROR = 1,      // I2C NACK／逾時連續超過 errMax
  BAD_CHIP_ID = 2,    // 0xD0 不是 0x61
  STUCK = 3,          // 原始溫壓 ADC 連續 stuckN 筆完全相同
  OUT_OF_RANGE = 4,   // 補償後溫壓連續 errMax 筆超出合理範圍
  MEAS_TIMEOUT = 5,   // 轉換逾時連續超過 errMax
  SELFTEST_FAIL = 6,  // 自測期間不合理
};

struct BaroOut {
  double t_s;
  uint32_t seq;
  bool valid;
  float press_Pa;
  float temp_C;
  float alt_m;
};

struct EnvOut {
  double t_s;
  uint32_t seq;
  bool valid;
  float humidity_pct;
  float gas_ohm;  // 加熱器未穩定或 gas_valid=0 時為 NaN
};

class IBme680Bus {
 public:
  virtual ~IBme680Bus() = default;
  virtual bool writeReg(uint8_t reg, uint8_t val) = 0;
  virtual bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) = 0;
};

class IBme680Sink {
 public:
  virtual ~IBme680Sink() = default;
  virtual void onBaro(const BaroOut& s) = 0;
  virtual void onEnv(const EnvOut& s) = 0;
  virtual void onStateChange(DrvState from, DrvState to) = 0;
  virtual void onFault(BaroFault code, uint32_t detail) = 0;
};

struct Bme680Config {
  float p0_Pa = 0.0f;           // 高度基準；0 = 自測期間取發射台平均
  uint16_t stuckN = 50;         // 約 1 s @50 Hz
  uint8_t errMax = 5;           // 連續錯誤門檻
  uint16_t gasPeriodTicks = 100;  // rg100Hz 下每 1 s 一次氣體轉換
  uint16_t heaterTemp_C = 320;  // 加熱器目標溫度
  uint16_t gasWait_ms = 30;     // datasheet：加熱器通常 20–30 ms 達溫
  uint8_t osrsT = 1;            // 暫存器碼：1 = x1
  uint8_t osrsP = 1;            // 1 = x1（讓 TP 轉換 < 10 ms，可在 rg100Hz 交替）
  uint8_t filter = 2;           // IIR 係數碼：2 = coef 3
  uint16_t selftestN = 10;      // 自測要連續幾筆合理樣本
  uint8_t measTimeoutTicks = 3; // 溫壓轉換最多等幾個 tick
  uint8_t gasTimeoutTicks = 10; // 溫壓＋氣體轉換最多等幾個 tick
  uint16_t faultRetryTicks = 100;  // FAULT 後多久重試
  uint8_t maxRetries = 3;
};

struct Bme680Calib {
  double t1, t2, t3;
  double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;
  double h1, h2, h3, h4, h5, h6, h7;
  double gh1, gh2, gh3;
  int resHeatRange;
  int resHeatVal;
  int rangeSwErr;
};

// 補償公式（Bosch BME68x SensorAPI 浮點版）
namespace Bme680Math {
bool parseCalib(const uint8_t* coeff42, Bme680Calib& c);
double compTemp(const Bme680Calib& c, uint32_t adc, double& tFine);
double compPress(const Bme680Calib& c, uint32_t adc, double tFine);
double compHum(const Bme680Calib& c, uint16_t adc, double tFine);
double compGas(const Bme680Calib& c, uint16_t adc, uint8_t range);
uint8_t heaterRes(const Bme680Calib& c, double target_C, double amb_C);
uint8_t gasWaitCode(uint16_t ms);
double altFromPress(double p_Pa, double p0_Pa);
}  // namespace Bme680Math

class Bme680Core {
 public:
  Bme680Core(IBme680Bus& bus, IBme680Sink& sink);
  void setConfig(const Bme680Config& cfg);
  const Bme680Config& config() const { return m_cfg; }
  // 每個 rg100Hz tick 呼叫一次
  void tick(double t_s);
  DrvState state() const { return m_state; }
  float p0_Pa() const { return m_p0; }
  uint32_t errorCount() const { return m_totalErrors; }
  uint32_t baroSeq() const { return m_baroSeq; }
  uint32_t envSeq() const { return m_envSeq; }

 private:
  void setState(DrvState s);
  void fault(BaroFault code, uint32_t detail);
  bool busErr();  // 回傳 true 表示已進 FAULT
  void tickInit();
  void tickMeasure(double t_s);
  bool trigger(bool gas);
  void handleData(const uint8_t* d, bool gas);

  IBme680Bus& m_bus;
  IBme680Sink& m_sink;
  Bme680Config m_cfg;
  Bme680Calib m_cal{};
  DrvState m_state = DrvState::OFF;
  uint8_t m_initStep = 0;
  uint8_t m_consecErr = 0;
  uint8_t m_consecRange = 0;
  uint8_t m_consecTimeout = 0;
  uint32_t m_totalErrors = 0;
  uint16_t m_faultTicks = 0;
  uint8_t m_retries = 0;
  bool m_measuring = false;
  bool m_measGas = false;
  bool m_humOn = false;
  bool m_gasOn = false;
  uint8_t m_waitTicks = 0;
  uint16_t m_ticksSinceGas = 0;
  double m_trigT = 0.0;
  double m_lastTemp_C = 25.0;
  uint32_t m_lastPAdc = 0xFFFFFFFF, m_lastTAdc = 0xFFFFFFFF;
  uint16_t m_stuckCount = 0;
  uint16_t m_selftestGood = 0;
  double m_p0Acc = 0.0;
  float m_p0 = 0.0f;
  uint32_t m_baroSeq = 0;
  uint32_t m_envSeq = 0;
};

}  // namespace Components
#endif
