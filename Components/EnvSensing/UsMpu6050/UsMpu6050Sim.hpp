// ======================================================================
// \title  UsMpu6050Sim.hpp
// \brief  UsMpu6050 Sim — Base ID 0x4280
// ======================================================================
#ifndef COMPONENTS_USMPU6050SIM_HPP
#define COMPONENTS_USMPU6050SIM_HPP

#include "Components/EnvSensing/UsMpu6050/UsMpu6050SimComponentAc.hpp"
#include "Components/EnvSensing/UsMpu6050/mpu6050_core.h"
#include <cstring>

namespace Components {

class MpuSimDevice {
 public:
  MpuSimDevice() { reset(); }
  void reset() {
    inject_ = 0;
    std::memset(accelRaw_, 0, sizeof(accelRaw_));
    std::memset(gyroRaw_, 0, sizeof(gyroRaw_));
    accelRaw_[2] = 2048;
  }
  void setInject(uint8_t c) { inject_ = c; }
  uint8_t inject() const { return inject_; }

  bool writeReg(uint8_t, uint8_t, const uint8_t*, uint8_t) {
    return inject_ != 3;
  }
  bool readReg(uint8_t, uint8_t reg, uint8_t* data, uint8_t len) {
    if (inject_ == 3) return false;
    std::memset(data, 0, len);
    if (reg == 0x75 && len >= 1) {
      data[0] = (inject_ == 4) ? static_cast<uint8_t>(0x68 ^ 0xFF) : 0x68;
      return true;
    }
    int16_t a[3] = {accelRaw_[0], accelRaw_[1], accelRaw_[2]};
    int16_t g[3] = {gyroRaw_[0], gyroRaw_[1], gyroRaw_[2]};
    if (inject_ == 2) g[0] = 32000;
    if (inject_ == 5) a[0] = 32000;
    if (reg == 0x3B && len >= 14) {
      for (int i = 0; i < 3; ++i) {
        data[2 * i] = static_cast<uint8_t>((a[i] >> 8) & 0xFF);
        data[2 * i + 1] = static_cast<uint8_t>(a[i] & 0xFF);
        data[8 + 2 * i] = static_cast<uint8_t>((g[i] >> 8) & 0xFF);
        data[8 + 2 * i + 1] = static_cast<uint8_t>(g[i] & 0xFF);
      }
    }
    return true;
  }

  ImuBusOps busOps() {
    ImuBusOps o;
    o.ctx = this;
    o.writeReg = &MpuSimDevice::writeThunk;
    o.readReg = &MpuSimDevice::readThunk;
    return o;
  }

 private:
  static bool writeThunk(void* ctx, uint8_t a, uint8_t r, const uint8_t* d, uint8_t n) {
    return static_cast<MpuSimDevice*>(ctx)->writeReg(a, r, d, n);
  }
  static bool readThunk(void* ctx, uint8_t a, uint8_t r, uint8_t* d, uint8_t n) {
    return static_cast<MpuSimDevice*>(ctx)->readReg(a, r, d, n);
  }
  uint8_t inject_;
  int16_t accelRaw_[3];
  int16_t gyroRaw_[3];
};

class UsMpu6050Sim final : public UsMpu6050SimComponentBase {
 public:
  explicit UsMpu6050Sim(const char* compName);
  ~UsMpu6050Sim() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void injectFault_handler(FwIndexType portNum, U8 code) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  void emitSample(const MpuSampleOut& s);

  MpuSimDevice m_dev;
  Mpu6050Core* m_core;
  U32 m_tlmDiv;
};

}  // namespace Components
#endif
