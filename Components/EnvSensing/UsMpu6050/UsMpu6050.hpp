#pragma once
#include "mpu6050_core.h"

namespace Components {

class UsMpu6050 {
 public:
  explicit UsMpu6050(ImuBusOps bus) : core_(mpu_core_create(bus)) {}
  ~UsMpu6050() { mpu_core_destroy(core_); }
  UsMpu6050(const UsMpu6050&) = delete;
  UsMpu6050& operator=(const UsMpu6050&) = delete;
  void configure(const MpuParams& p) { mpu_core_set_params(core_, &p); mpu_core_reset(core_); }
  void schedIn_handler(double t_s, MpuSampleOut& out) { mpu_core_tick(core_, t_s, &out); }
  Mpu6050Core* core() { return core_; }
 private:
  Mpu6050Core* core_;
};

class UsMpu6050Sim {
 public:
  explicit UsMpu6050Sim(ImuBusOps bus) : core_(mpu_core_create(bus)), inject_(0) {}
  ~UsMpu6050Sim() { mpu_core_destroy(core_); }
  void configure(const MpuParams& p) { mpu_core_set_params(core_, &p); mpu_core_reset(core_); }
  void injectFault_handler(uint8_t code) { inject_ = code; }
  void schedIn_handler(double t_s, MpuSampleOut& out) { mpu_core_tick(core_, t_s, &out); }
  uint8_t injectCode() const { return inject_; }
 private:
  Mpu6050Core* core_;
  uint8_t inject_;
};

}  // namespace Components
