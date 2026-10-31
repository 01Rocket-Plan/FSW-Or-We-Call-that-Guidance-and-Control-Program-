// BaroDriver：BME680 的 F´ 元件外殼（I2C 走 Drv.I2c 埠；邏輯在 Bme680Core）
#ifndef ENVSENSING_BARODRIVER_HPP
#define ENVSENSING_BARODRIVER_HPP

#include "Components/EnvSensing/BaroDriver/BaroDriverComponentAc.hpp"
#include "Components/EnvSensing/BaroDriver/Bme680Core.hpp"

namespace Components {

class BaroDriver final : public BaroDriverComponentBase, private IBme680Bus, private IBme680Sink {
 public:
  explicit BaroDriver(const char* compName);
  ~BaroDriver() override = default;
  //! 拓樸設定：I2C 位址（SDO 接地 0x76，接 VDD 0x77）
  void configure(U32 i2cAddr);

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  bool writeReg(uint8_t reg, uint8_t val) override;
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) override;
  void onBaro(const BaroOut& s) override;
  void onEnv(const EnvOut& s) override;
  void onStateChange(DrvState from, DrvState to) override;
  void onFault(BaroFault code, uint32_t detail) override;

  void loadParams();
  void writeTlm();

  Bme680Core m_core;
  U32 m_addr;
  U32 m_tlmDiv;
};

}  // namespace Components
#endif
