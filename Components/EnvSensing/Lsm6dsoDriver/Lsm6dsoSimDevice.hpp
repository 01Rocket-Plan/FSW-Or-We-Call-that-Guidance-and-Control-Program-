// ======================================================================
// \title  Lsm6dsoSimDevice.hpp
// \brief  LSM6DSO 暫存器級模擬（給 Lsm6dsoDriverSim／SIL 用；跟真機走同一顆 lsm6dso_core）
//         注入碼（跟 ImuSimDevice 對齊）：0 正常、1 卡值、2 陀螺飽和、3 匯流排錯、
//         4 WHO_AM_I 錯、5 加速度飽和、6 自測響應太弱
// ======================================================================
#pragma once
#include "lsm6dso_core.h"
#include "Lsm6dsoRegs.h"
#include <cstdint>
#include <cstring>

namespace Components {

class Lsm6dsoSimDevice {
 public:
  Lsm6dsoSimDevice() { reset(); }

  void reset() {
    std::memset(regs_, 0, sizeof(regs_));
    regs_[LSM_REG_CTRL3_C] = LSM_CTRL3_IF_INC;  // 上電預設 IF_INC=1
    inject_ = 0;
    dither_ = 0;
    // 真值（晶片軸）：晶片 Z 沿鼻，墊上靜置 ≈ +1 g；經預設 DCM 變機體 +X
    accTruth_mg_[0] = 0.0f; accTruth_mg_[1] = 0.0f; accTruth_mg_[2] = 1000.0f;
    gyroTruth_mdps_[0] = gyroTruth_mdps_[1] = gyroTruth_mdps_[2] = 0.0f;
  }

  void setInject(uint8_t code) { inject_ = code; }
  uint8_t inject() const { return inject_; }

  void setTruth(const float acc_mg[3], const float gyro_mdps[3]) {
    for (int i = 0; i < 3; ++i) { accTruth_mg_[i] = acc_mg[i]; gyroTruth_mdps_[i] = gyro_mdps[i]; }
  }

  bool writeReg(uint8_t /*addr*/, uint8_t reg, const uint8_t* data, uint8_t len) {
    if (inject_ == 3 || !data || len < 1) return false;
    if (reg == LSM_REG_CTRL3_C && (data[0] & LSM_CTRL3_SW_RESET)) {
      const uint8_t inj = inject_;
      reset();  // SW_RESET：暫存器回預設，位元自己清掉
      inject_ = inj;
      return true;
    }
    regs_[reg] = data[0];
    return true;
  }

  bool readReg(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    if (inject_ == 3 || !data) return false;
    std::memset(data, 0, len);
    if (reg == LSM_REG_WHO_AM_I && len >= 1) {
      data[0] = (addr == LSM6DSO_I2C_ADDR_SA0_LOW) ? static_cast<uint8_t>(LSM6DSO_WHO_AM_I_VALUE) : 0x00;
      if (inject_ == 4) data[0] ^= 0xFF;
      return true;
    }
    if (reg == LSM_REG_OUTX_L_G && len >= 12) {
      int16_t g[3], a[3];
      const uint8_t c5 = regs_[LSM_REG_CTRL5_C];
      const float stXl = (inject_ == 6) ? 10.0f : 500.0f;         // mg（規格 50–1700）
      const float stG = (inject_ == 6) ? 10000.0f : 300000.0f;    // mdps（規格 150–700 dps）
      if (inject_ != 1) dither_ ^= 1;  // ±1 LSB 抖動，避免 SIL 被判卡值；注入 1＝凍結
      for (int i = 0; i < 3; ++i) {
        float am = accTruth_mg_[i] + (((c5 & 0x03) == LSM_CTRL5_ST_XL_POS) ? stXl : 0.0f);
        float gm = gyroTruth_mdps_[i] + (((c5 & 0x0C) == LSM_CTRL5_ST_G_POS) ? stG : 0.0f);
        a[i] = sat16(am / xlSens() + (i == 1 ? dither_ : 0));
        g[i] = sat16(gm / gSens() + (i == 1 ? dither_ : 0));
      }
      if (inject_ == 2) g[0] = 32767;  // 削頂（±2000 dps 檔實際上限 ≈ 2294 dps）
      if (inject_ == 5) a[0] = 32767;  // 削頂（±16 g 檔 ≈ 15.99 g）
      for (int i = 0; i < 3; ++i) {
        data[2 * i] = static_cast<uint8_t>(static_cast<uint16_t>(g[i]) & 0xFF);
        data[2 * i + 1] = static_cast<uint8_t>((static_cast<uint16_t>(g[i]) >> 8) & 0xFF);
        data[6 + 2 * i] = static_cast<uint8_t>(static_cast<uint16_t>(a[i]) & 0xFF);
        data[6 + 2 * i + 1] = static_cast<uint8_t>((static_cast<uint16_t>(a[i]) >> 8) & 0xFF);
      }
      return true;
    }
    if (len >= 1) data[0] = regs_[reg];
    return true;
  }

  LsmBusOps busOps() {
    LsmBusOps o;
    o.ctx = this;
    o.writeReg = &Lsm6dsoSimDevice::writeThunk;
    o.readReg = &Lsm6dsoSimDevice::readThunk;
    return o;
  }

 private:
  // CTRL1_XL FS_XL[3:2]：00 ±2 g、01 ±16 g、10 ±4 g、11 ±8 g（XL_FS_MODE=0）
  float xlSens() const {
    switch ((regs_[LSM_REG_CTRL1_XL] >> 2) & 0x03) {
      case 1: return 0.488f;
      case 2: return 0.122f;
      case 3: return 0.244f;
      default: return 0.061f;
    }
  }
  // CTRL2_G FS[3:1]：000 250、001 125、010 500、100 1000、110 2000 dps
  float gSens() const {
    switch ((regs_[LSM_REG_CTRL2_G] >> 1) & 0x07) {
      case 1: return 4.375f;
      case 2: return 17.50f;
      case 4: return 35.0f;
      case 6: return 70.0f;
      default: return 8.75f;
    }
  }
  static int16_t sat16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return static_cast<int16_t>(v);
  }
  static bool writeThunk(void* ctx, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
    return static_cast<Lsm6dsoSimDevice*>(ctx)->writeReg(addr, reg, data, len);
  }
  static bool readThunk(void* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    return static_cast<Lsm6dsoSimDevice*>(ctx)->readReg(addr, reg, data, len);
  }

  uint8_t regs_[0x80];
  uint8_t inject_;
  int dither_;
  float accTruth_mg_[3];
  float gyroTruth_mdps_[3];
};

}  // namespace Components
