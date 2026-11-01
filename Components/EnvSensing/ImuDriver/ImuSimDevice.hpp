#pragma once
#include "imu_core.h"
#include <cstdint>
#include <cstring>

namespace Components {

/** Register-level ICM sim for ImuDriverSim / host-style inject codes. */
class ImuSimDevice {
 public:
  ImuSimDevice() { reset(); }

  void reset() {
    bank_ = 0;
    gyroSt_ = 0;
    accelSt_ = 0;
    inject_ = 0;
    std::memset(accelRaw_, 0, sizeof(accelRaw_));
    std::memset(gyroRaw_, 0, sizeof(gyroRaw_));
    accelRaw_[2] = 2048;  // ~1 g at ±16 g
  }

  void setInject(uint8_t code) { inject_ = code; }
  uint8_t inject() const { return inject_; }

  void setTruth(const int16_t accel[3], const int16_t gyro[3]) {
    for (int i = 0; i < 3; ++i) {
      accelRaw_[i] = accel[i];
      gyroRaw_[i] = gyro[i];
    }
  }

  bool writeReg(uint8_t /*addr*/, uint8_t reg, const uint8_t* data, uint8_t /*len*/) {
    if (inject_ == 3) return false;  // bus err
    if (reg == 0x7F && data) bank_ = static_cast<uint8_t>((data[0] >> 4) & 3);
    if (bank_ == 2 && reg == 0x02 && data) gyroSt_ = data[0];
    if (bank_ == 2 && reg == 0x15 && data) accelSt_ = data[0];
    return true;
  }

  bool readReg(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    if (inject_ == 3) return false;
    std::memset(data, 0, len);
    if (bank_ == 0 && reg == 0x00 && len >= 1) {
      data[0] = (addr == 0x69) ? 0xEA : 0xE0;
      if (inject_ == 4) data[0] ^= 0xFF;
      return true;
    }
    if (bank_ == 1 && len >= 1 &&
        (reg == 0x02 || reg == 0x03 || reg == 0x04 || reg == 0x0E || reg == 0x0F || reg == 0x10)) {
      data[0] = 100;  // OTP code
      return true;
    }
    int16_t a[3] = {accelRaw_[0], accelRaw_[1], accelRaw_[2]};
    int16_t g[3] = {gyroRaw_[0], gyroRaw_[1], gyroRaw_[2]};
    if (inject_ == 2) g[0] = 32000;
    if (inject_ == 5) a[0] = 32000;
    const int delta = (inject_ == 6) ? 10 : 7000;
    if (gyroSt_ & 0x38) {
      for (int i = 0; i < 3; ++i) g[i] = static_cast<int16_t>(g[i] + delta);
    }
    if (accelSt_ & 0x1C) {
      for (int i = 0; i < 3; ++i) a[i] = static_cast<int16_t>(a[i] + delta);
    }
    if (bank_ == 0 && reg == 0x2D && len >= 12) {
      for (int i = 0; i < 3; ++i) {
        data[2 * i] = static_cast<uint8_t>((a[i] >> 8) & 0xFF);
        data[2 * i + 1] = static_cast<uint8_t>(a[i] & 0xFF);
        data[6 + 2 * i] = static_cast<uint8_t>((g[i] >> 8) & 0xFF);
        data[6 + 2 * i + 1] = static_cast<uint8_t>(g[i] & 0xFF);
      }
    }
    return true;
  }

  ImuBusOps busOps() {
    ImuBusOps o;
    o.ctx = this;
    o.writeReg = &ImuSimDevice::writeThunk;
    o.readReg = &ImuSimDevice::readThunk;
    return o;
  }

 private:
  static bool writeThunk(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
    return static_cast<ImuSimDevice*>(ctx)->writeReg(addr, reg, data, len);
  }
  static bool readThunk(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    return static_cast<ImuSimDevice*>(ctx)->readReg(addr, reg, data, len);
  }

  uint8_t bank_;
  uint8_t gyroSt_;
  uint8_t accelSt_;
  uint8_t inject_;
  int16_t accelRaw_[3];
  int16_t gyroRaw_[3];
};

}  // namespace Components
