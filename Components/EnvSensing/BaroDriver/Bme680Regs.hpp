// BME680 暫存器（datasheet BST-BME680-DS001 與 Bosch BME68x SensorAPI）
#ifndef ENVSENSING_BME680REGS_HPP
#define ENVSENSING_BME680REGS_HPP
#include <cstdint>
namespace Components {
namespace Bme680Reg {
constexpr uint8_t I2C_ADDR_PRIMARY = 0x77;  // SDO 接 VDD（gate C12 已確認 0x77）
constexpr uint8_t I2C_ADDR_SECONDARY = 0x77;
constexpr uint8_t CHIP_ID = 0xD0;
constexpr uint8_t CHIP_ID_VAL = 0x61;
constexpr uint8_t RESET = 0xE0;
constexpr uint8_t RESET_CMD = 0xB6;
constexpr uint8_t COEFF1 = 0x8A;  constexpr uint8_t COEFF1_LEN = 23;
constexpr uint8_t COEFF2 = 0xE1;  constexpr uint8_t COEFF2_LEN = 14;
constexpr uint8_t COEFF3 = 0x00;  constexpr uint8_t COEFF3_LEN = 5;
constexpr uint8_t CTRL_HUM = 0x72;    // bit2:0 osrs_h（需在 CTRL_MEAS 前寫）
constexpr uint8_t CTRL_MEAS = 0x74;   // bit7:5 osrs_t，bit4:2 osrs_p，bit1:0 mode
constexpr uint8_t CONFIG = 0x75;      // bit4:2 filter
constexpr uint8_t FIELD0 = 0x1D;      // 0x1D..0x2B 共 15 B
constexpr uint8_t FIELD0_LEN = 15;
constexpr uint8_t MODE_FORCED = 0x01;
constexpr uint8_t STATUS_NEW_DATA = 0x80;
constexpr uint8_t STATUS_MEASURING = 0x20;
}  // namespace Bme680Reg
}  // namespace Components
#endif
