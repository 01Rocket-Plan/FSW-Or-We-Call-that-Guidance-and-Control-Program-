// Ov7670Regs.hpp：OV7670 暫存器（SCCB：8-bit 位址、8-bit 資料）
// 來源：OmniVision OV7670/OV7171 CMOS VGA datasheet v1.4 暫存器表；
//       QCIF／YUV 設定值對照 Linux drivers/media/i2c/ov7670.c。冷測時再對一次規格書（見 README「待冷測核對」）。
#ifndef COMPONENTS_CAMERA_OV7670DRIVER_OV7670REGS_HPP
#define COMPONENTS_CAMERA_OV7670DRIVER_OV7670REGS_HPP

#include <cstdint>

namespace Components {
namespace Ov7670Regs {

//! SCCB 7-bit 從機位址（寫 0x42／讀 0x43）
static constexpr uint8_t SCCB_ADDR_7BIT = 0x21;

// 識別
static constexpr uint8_t PID = 0x0A;   //!< 產品 ID 高位，固定 0x76
static constexpr uint8_t VER = 0x0B;   //!< 產品 ID 低位，固定 0x73
static constexpr uint8_t MIDH = 0x1C;  //!< 製造商 ID 高位 0x7F
static constexpr uint8_t MIDL = 0x1D;  //!< 製造商 ID 低位 0xA2
static constexpr uint8_t EXPECTED_PID = 0x76;
static constexpr uint8_t EXPECTED_VER = 0x73;
static constexpr uint16_t EXPECTED_CHIP_ID = 0x7673;

// 設定
static constexpr uint8_t CLKRC = 0x11;      //!< 內部時鐘預除
static constexpr uint8_t COM7 = 0x12;       //!< bit7 軟重置；bit4 QVGA；bit3 QCIF；bit2 RGB
static constexpr uint8_t COM3 = 0x0C;       //!< bit2 DCW（縮放）致能
static constexpr uint8_t TSLB = 0x3A;       //!< 輸出序列
static constexpr uint8_t COM14 = 0x3E;      //!< PCLK 分頻／手動縮放
static constexpr uint8_t COM15 = 0x40;      //!< 輸出範圍＋RGB565
static constexpr uint8_t RGB444 = 0x8C;     //!< 關閉 RGB444
static constexpr uint8_t SCALING_XSC = 0x70;
static constexpr uint8_t SCALING_YSC = 0x71;
static constexpr uint8_t SCALING_DCWCTR = 0x72;
static constexpr uint8_t SCALING_PCLK_DIV = 0x73;
static constexpr uint8_t SCALING_PCLK_DELAY = 0xA2;

static constexpr uint8_t COM7_RESET = 0x80;
//! COM7：bit3 QCIF、bit2／bit0＝0 → YUV 輸出
static constexpr uint8_t COM7_QCIF_YUV = 0x08;
//! COM15：bit7..6＝11 輸出範圍 00–FF；bit5..4＝00（不是 RGB565／RGB555）
static constexpr uint8_t COM15_YUV_FULL = 0xC0;

//! 輸出（Edward 2026-11-14 PR #55：上級 1 fps、144p＋灰階）：
//!   感測器出 QCIF 176×144 YUV422（YUYV，每像素 2 B）；驅動只取 Y（亮度）→ 灰階每像素 1 B。
//!   OV7670 沒有 JPEG 編碼器，輸出是原始像素。
static constexpr uint16_t FRAME_WIDTH = 176;
static constexpr uint16_t FRAME_HEIGHT = 144;
//! 感測器 DVP 一幀（YUYV）：176×144×2＝50,688 B
static constexpr uint32_t RAW_BYTES_PER_PIXEL = 2;
static constexpr uint32_t RAW_FRAME_BYTES = 176u * 144u * 2u;
//! 送進 SD 的灰階一幀（只有 Y）：176×144×1＝25,344 B（2 KB 分塊＝13 塊）
static constexpr uint32_t FRAME_BYTES = 176u * 144u;

}  // namespace Ov7670Regs
}  // namespace Components

#endif
