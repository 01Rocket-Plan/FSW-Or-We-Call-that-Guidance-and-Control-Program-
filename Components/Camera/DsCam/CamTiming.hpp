// CamTiming：下級相機 DsCam（ESP32-CAM 模組，內建感測器 OV2640，640×480）時序模型與預設參數。
// 相機型號（首席 2026-11-18 確認，不要混用）：
//   - 下級相機 DsCam ＝ ESP32-CAM 模組；OV2640 是這個模組內建的感測器 → 本檔只用 OV2640 的規格。
//   - 上級（主）相機 UsCam ＝ OV7670（Components/Camera/Ov7670Driver，PR #55），沒有硬體 JPEG，不適用本檔。
//   - OV3640 只是停在 fork 分支的備援，不在 main，不適用本檔。
// 解析度（Edward 2026-11-24 決議）：DsCam 完整錄影維持 640×480 JPEG。
//   - 「144p 灰階 1 fps」是上級 UsCam（OV7670，PR #55，Ov7670Driver.fpp:3），不是 DsCam。
//   - 「144P（256×144）1～3 FPS」是 DsCam 的地面預覽 IGroundPreviewLink（CamLinkPacket.md），
//     不是錄影；預覽尚未接線，本檔不模擬。
// ⚠ 本檔所有 I2C／SCCB 時間都是「規格書計算值，未實測」；實測待 HIL（相機板上機量 SCL 波形）。
//   OV2640 的 SCCB 數字只適用 ESP32-CAM 模組內部（ESP32↔OV2640）；上級 OV7670 的 SCCB 時序未建模、
//   不給數字（待 HIL）。
// 純 constexpr：不配置記憶體、不讀系統時間、不 sleep。數字全部有出處；相機組可再改。
//
// 出處：
//   [UM10204] NXP, "I2C-bus specification and user manual", UM10204 Rev. 7.0（2021-10-01）
//             §3.1.6：每個位元組後面跟 1 個 ACK／NACK 位元 → 每位元組佔 9 個 SCL 週期；
//             §3.1.10：交易第一個位元組是 7-bit 位址＋R/W。
//   [SCCB]    OmniVision, "Serial Camera Control Bus Functional Specification"：寫暫存器是
//             3-phase write（ID 位址、子位址、資料），每個 phase 9 位元（8 資料＋1 Don't-Care）。
//   [OV2640]  OmniVision, "OV2640 Color CMOS UXGA (2.0 MegaPixel) CameraChip with OmniPixel2
//             Technology", Datasheet Version 1.6（2006-02-28）
//             p.1 "Maximum Image Transfer Rate"：UXGA/SXGA 15 fps、SVGA 30 fps、CIF 60 fps；
//             Table 8：XVCLK 6 MHz（min）、24 MHz（typ）；Table 9：SCCB fSIO_C 最大 400 kHz。
//   [ESPCAM]  espressif/esp32-camera（commit b3f7fe8，2026-11-09）
//             README.md 範例 camera_config：xclk_freq_hz = 20000000、jpeg_quality = 12
//             （註解 "0-63, for OV series camera sensors, lower number means higher quality"）；
//             driver/include/esp_camera.h：jpeg_quality "Quality of JPEG output. 0-63 lower means
//             higher quality"（同檔範例用 10）；Kconfig：SCCB_CLK_FREQ default 100000、range
//             100000–400000；sensors/ov2640.c set_framesize()：framesize ≤ SVGA（含 VGA 640×480）
//             走 OV2640_MODE_SVGA。
//
// 沒有出處、刻意不給數字的（待相機組冷測量）：
//   - JPEG 一幀的位元組數（看畫面內容與 jpeg_quality；sim 仍用 Esp32CamSimDevice 的 1024 B 占位）
//   - 20 MHz XCLK 下 OV2640 實際幀率（datasheet 只給最大幀率，沒寫對應 XVCLK）
#ifndef COMPONENTS_CAMERA_DSCAM_CAMTIMING_HPP
#define COMPONENTS_CAMERA_DSCAM_CAMTIMING_HPP

#include <cstdint>

namespace Components {
namespace CamTiming {

// ---- I2C／SCCB 位元框架（規格書計算值，未實測；實測待 HIL）----
//! [UM10204 §3.1.6] 8 資料位元＋1 ACK；[SCCB] 8 資料位元＋1 Don't-Care
constexpr uint32_t kBusClocksPerByte = 9;
//! [UM10204 §3.1.10] 一筆交易開頭的位址位元組數（7-bit 位址＋R/W）
constexpr uint32_t kI2cAddressBytes = 1;
//! [SCCB] 3-phase write：ID 位址＋子位址＋資料（OV2640 暫存器位址 8 bit）
constexpr uint32_t kSccbWritePhases = 3;

// ---- 預設值 ----
//! 飛控↔ESP32-CAM ICamLink 的 I2C 時脈：repo 既有預設（Esp32CamCore::kDefaultI2cClockHz、
//! Esp32Cam.fpp I2C_CLOCK_HZ）＝ 400 kHz Fast-mode 上限 [UM10204]
constexpr uint32_t kCamLinkI2cClockHz = 400000;
//! ESP32-CAM 模組內部 ESP32↔OV2640 SCCB 時脈（只適用 DsCam；OV7670 未建模）：[ESPCAM] Kconfig SCCB_CLK_FREQ 預設 100 kHz（相機組可再改）
constexpr uint32_t kSccbClockHzDefault = 100000;
//! [OV2640 Table 9] SCCB 最大時脈 400 kHz
constexpr uint32_t kOv2640SccbClockHzMax = 400000;
//! [ESPCAM] README 範例 xclk_freq_hz（相機組可再改；OV2640 Table 8 範圍 6 MHz 起、典型 24 MHz）
constexpr uint32_t kXclkHzDefault = 20000000;
//! [ESPCAM] README 範例 jpeg_quality = 12（0–63，越小畫質越好）。相機組可再改。
//! 注意：這不是 libjpeg／PIL 的 0–100 品質刻度，兩者不能直接換算。
constexpr uint8_t kJpegQualityDefault = 12;
constexpr uint8_t kJpegQualityMin = 0;
constexpr uint8_t kJpegQualityMax = 63;
//! [OV2640 p.1] SVGA 模式最大 30 fps；640×480 走 SVGA 模式 [ESPCAM ov2640.c]
constexpr uint32_t kOv2640SvgaMaxFps = 30;
//! 下級相機解析度（repo：Esp32CamSimDevice::pushFrame 640×480）
constexpr uint16_t kFrameWidth = 640;
constexpr uint16_t kFrameHeight = 480;

// ---- 時序公式 ----
//! I2C 交易的 SCL 週期數：(位址 1 B ＋ 資料 n B) × 9
constexpr uint64_t i2cTransactionClocks(uint32_t dataBytes) {
  return static_cast<uint64_t>(kI2cAddressBytes + dataBytes) * kBusClocksPerByte;
}
//! I2C 交易時間（ns）下限：只算 SCL 週期，不含 START／STOP 建立保持時間與 clock stretching
constexpr uint64_t i2cTransactionTimeNs(uint32_t dataBytes, uint32_t clockHz) {
  return (clockHz == 0u) ? 0u : i2cTransactionClocks(dataBytes) * 1000000000ull / clockHz;
}
//! SCCB 寫一個暫存器的 SCL 週期數：3 phase × 9
constexpr uint64_t sccbRegWriteClocks() {
  return static_cast<uint64_t>(kSccbWritePhases) * kBusClocksPerByte;
}
//! SCCB 寫 n 個暫存器的時間（ns）下限（同上，不含 START／STOP 與暫存器之間的間隔）
constexpr uint64_t sccbRegWriteTimeNs(uint32_t nRegs, uint32_t clockHz) {
  return (clockHz == 0u) ? 0u : sccbRegWriteClocks() * nRegs * 1000000000ull / clockHz;
}
//! 感測器最短幀間隔（ns）＝ 1 / 最大幀率
constexpr uint64_t sensorMinFramePeriodNs(uint32_t maxFps) {
  return (maxFps == 0u) ? 0u : 1000000000ull / maxFps;
}
//! jpeg_quality 是否在 esp32-camera 允許範圍 0–63
constexpr bool jpegQualityValid(uint32_t q) { return q <= kJpegQualityMax; }

}  // namespace CamTiming
}  // namespace Components
#endif
