#pragma once
#include <cstdint>

namespace EnvSensing {
namespace ImuRegs {

// ICM-20948 / ICM-20648 (banked)
constexpr uint8_t REG_BANK_SEL     = 0x7F;
constexpr uint8_t BANK0_WHO_AM_I   = 0x00;
constexpr uint8_t BANK0_PWR_MGMT_1 = 0x06;
constexpr uint8_t BANK0_PWR_MGMT_2 = 0x07;
constexpr uint8_t BANK0_ACCEL_XOUT_H = 0x2D;
constexpr uint8_t BANK0_GYRO_XOUT_H  = 0x33;

// Bank 1 — factory self-test OTP codes
constexpr uint8_t BANK1_SELF_TEST_X_GYRO  = 0x02;
constexpr uint8_t BANK1_SELF_TEST_Y_GYRO  = 0x03;
constexpr uint8_t BANK1_SELF_TEST_Z_GYRO  = 0x04;
constexpr uint8_t BANK1_SELF_TEST_X_ACCEL = 0x0E;
constexpr uint8_t BANK1_SELF_TEST_Y_ACCEL = 0x0F;
constexpr uint8_t BANK1_SELF_TEST_Z_ACCEL = 0x10;

constexpr uint8_t BANK2_GYRO_CONFIG_1  = 0x01;
constexpr uint8_t BANK2_GYRO_CONFIG_2  = 0x02;
constexpr uint8_t BANK2_GYRO_SMPLRT_DIV = 0x00;
constexpr uint8_t BANK2_ACCEL_CONFIG   = 0x14;
constexpr uint8_t BANK2_ACCEL_CONFIG_2 = 0x15;
constexpr uint8_t BANK2_ACCEL_SMPLRT_DIV_1 = 0x10;
constexpr uint8_t BANK2_ACCEL_SMPLRT_DIV_2 = 0x11;

constexpr uint8_t WHOAMI_ICM20948 = 0xEA;
constexpr uint8_t WHOAMI_ICM20648 = 0xE0;

constexpr uint8_t PWR1_DEVICE_RESET = 0x80;
constexpr uint8_t PWR1_SLEEP        = 0x40;
constexpr uint8_t PWR1_CLKSEL_AUTO  = 0x01;

// Flight ranges
constexpr uint8_t GYRO_CFG1_2000DPS_DLPF = 0x07;  // FS=3, DLPF on
constexpr uint8_t ACCEL_CFG_16G_DLPF     = 0x07;  // FS=3, DLPF on
// Self-test ranges (FS=0): ±250 dps / ±2 g
constexpr uint8_t GYRO_CFG1_250DPS_DLPF  = 0x01;
constexpr uint8_t ACCEL_CFG_2G_DLPF      = 0x01;
// ST enable masks (XG/YG/ZG_ST = bits 5:3; XA/YA/ZA_ST = bits 4:2 on ACCEL_CONFIG_2)
constexpr uint8_t GYRO_ST_ENABLE_XYZ  = 0x38;
constexpr uint8_t ACCEL_ST_ENABLE_XYZ = 0x1C;

constexpr float GYRO_SENS_2000_DPS = 16.4f;
constexpr float ACCEL_SENS_16G     = 2048.0f;
constexpr float GYRO_SENS_250_DPS  = 131.0f;
constexpr float ACCEL_SENS_2G      = 16384.0f;
constexpr float DEG2RAD = 0.017453292519943295f;
constexpr float G0_MPS2 = 9.80665f;

// MPU6050
constexpr uint8_t MPU_SMPLRT_DIV   = 0x19;
constexpr uint8_t MPU_CONFIG       = 0x1A;
constexpr uint8_t MPU_GYRO_CONFIG  = 0x1B;
constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_PWR_MGMT_1   = 0x6B;
constexpr uint8_t MPU_WHO_AM_I     = 0x75;
constexpr uint8_t WHOAMI_MPU6050   = 0x68;
constexpr uint8_t MPU_PWR1_RESET   = 0x80;
constexpr uint8_t MPU_PWR1_WAKE_PLL = 0x01;
constexpr uint8_t MPU_GYRO_FS_2000 = 0x18;
constexpr uint8_t MPU_ACCEL_FS_16G = 0x18;

}  // namespace ImuRegs
}  // namespace EnvSensing
