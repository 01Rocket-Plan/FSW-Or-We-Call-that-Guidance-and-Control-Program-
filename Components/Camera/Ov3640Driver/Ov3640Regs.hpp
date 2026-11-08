// Ov3640Regs.hpp：OV3640 暫存器位址（16-bit 位址、8-bit 資料，透過 SCCB）
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_OV3640REGS_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_OV3640REGS_HPP

#include <cstdint>

namespace Components {
namespace Ov3640Regs {

static constexpr uint16_t PIDH = 0x300A;
static constexpr uint16_t PIDL = 0x300B;
static constexpr uint16_t MIDH = 0x300C;
static constexpr uint16_t MIDL = 0x300D;

static constexpr uint8_t EXPECTED_PIDH = 0x36;
static constexpr uint8_t EXPECTED_PIDL = 0x4C;
static constexpr uint16_t EXPECTED_CHIP_ID = 0x364C;

}  // namespace Ov3640Regs
}  // namespace Components

#endif
