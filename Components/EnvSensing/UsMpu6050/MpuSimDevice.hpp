// MpuSimDevice：MPU6050 暫存器層模擬（給 UsMpu6050Sim 與 host 測試）
// 預設靜置：accel Z = 2048 LSB ＝ ±16 g 檔的 1 g，gyro = 0。
// setTruth(accel_raw[3], gyro_raw[3]) 餵入原始 ADC，讓 mpu6050_core 走完整讀取路徑。
#ifndef ENVSENSING_MPUSIMDEVICE_HPP
#define ENVSENSING_MPUSIMDEVICE_HPP

#include "Components/EnvSensing/ImuDriver/imu_core.h"
#include <cstdint>
#include <cstring>

namespace Components {

class MpuSimDevice {
 public:
  MpuSimDevice() { reset(); }

  void reset() {
    inject_ = 0;
    std::memset(accelRaw_, 0, sizeof(accelRaw_));
    std::memset(gyroRaw_, 0, sizeof(gyroRaw_));
    accelRaw_[2] = 2048;  // 1 g at ACCEL_FS_16G (2048 LSB/g)
  }

  //! 餵入感測器原生軸原始值（與 MPU_ACCEL_XOUT_H / GYRO_XOUT_H 相同的 int16）。
  void setTruth(const int16_t accel_raw[3], const int16_t gyro_raw[3]) {
    if (accel_raw == nullptr || gyro_raw == nullptr) {
      return;
    }
    for (int i = 0; i < 3; ++i) {
      accelRaw_[i] = accel_raw[i];
      gyroRaw_[i] = gyro_raw[i];
    }
  }

  void getTruth(int16_t accel_raw[3], int16_t gyro_raw[3]) const {
    if (accel_raw == nullptr || gyro_raw == nullptr) {
      return;
    }
    for (int i = 0; i < 3; ++i) {
      accel_raw[i] = accelRaw_[i];
      gyro_raw[i] = gyroRaw_[i];
    }
  }

  void setInject(uint8_t c) { inject_ = c; }
  uint8_t inject() const { return inject_; }

  bool writeReg(uint8_t, uint8_t, const uint8_t*, uint8_t) { return inject_ != 3; }

  bool readReg(uint8_t, uint8_t reg, uint8_t* data, uint8_t len) {
    if (inject_ == 3) {
      return false;
    }
    std::memset(data, 0, len);
    if (reg == 0x75 && len >= 1) {
      data[0] = (inject_ == 4) ? static_cast<uint8_t>(0x68 ^ 0xFF) : 0x68;
      return true;
    }
    int16_t a[3] = {accelRaw_[0], accelRaw_[1], accelRaw_[2]};
    int16_t g[3] = {gyroRaw_[0], gyroRaw_[1], gyroRaw_[2]};
    if (inject_ == 2) {
      g[0] = 32000;
    }
    if (inject_ == 5) {
      a[0] = 32000;
    }
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

}  // namespace Components
#endif
