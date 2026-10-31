// ======================================================================
// \title  ImuDriver.hpp
// \author 環境感測系統組子Bot1
// \brief  ICM-20948 主 / ICM-20648 備援 — 邏輯在 imu_core（C，可 host 單測）
//
// 完整 F´ 建置需 lib/fprime 後由 fpp-to-cpp 產生 ComponentBase。
// ======================================================================
#pragma once
#include "imu_core.h"

namespace Components {

class ImuDriver {
 public:
  explicit ImuDriver(ImuBusOps bus) : core_(imu_core_create(bus)) {}
  ~ImuDriver() { imu_core_destroy(core_); }
  ImuDriver(const ImuDriver&) = delete;
  ImuDriver& operator=(const ImuDriver&) = delete;

  void configure(const ImuParams& p) {
    imu_core_set_params(core_, &p);
    imu_core_reset(core_);
  }

  /** 1 kHz schedIn；t_s 只來自 F´ 時間服務 */
  void schedIn_handler(double t_s, ImuSampleOut& out) {
    imu_core_tick(core_, t_s, &out);
  }

  ImuCore* core() { return core_; }

 private:
  ImuCore* core_;
};

class ImuDriverSim {
 public:
  explicit ImuDriverSim(ImuBusOps bus) : core_(imu_core_create(bus)), inject_(0) {}
  ~ImuDriverSim() { imu_core_destroy(core_); }
  void configure(const ImuParams& p) {
    imu_core_set_params(core_, &p);
    imu_core_reset(core_);
  }
  void injectFault_handler(uint8_t code) { inject_ = code; }
  void schedIn_handler(double t_s, ImuSampleOut& out) { imu_core_tick(core_, t_s, &out); }
  uint8_t injectCode() const { return inject_; }
  ImuCore* core() { return core_; }
 private:
  ImuCore* core_;
  uint8_t inject_;
};

}  // namespace Components
