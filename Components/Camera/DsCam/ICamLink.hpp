// ICamLink：ESP32-CAM 與 DS 飛控之間的抽象位元組鏈路（實體層為 I2C）
// 規則：純介面、無實體層實作；腳位之後對齊 Anatoly 腳位表，禁止在此擅自指定。
// 本鏈路只傳指令、狀態與幀數，禁止傳影像像素。
#ifndef COMPONENTS_CAMERA_DSCAM_ICAMLINK_HPP
#define COMPONENTS_CAMERA_DSCAM_ICAMLINK_HPP

#include <cstdint>

namespace Components {

class ICamLink {
 public:
  virtual ~ICamLink() = default;
  // 非阻塞送出；回傳真實送出的位元組數（可能 < len）。
  virtual uint32_t send(const uint8_t* buf, uint32_t len) = 0;
  // 非阻塞接收；回傳實際讀到的位元組數（0 表示目前無資料）。
  virtual uint32_t recv(uint8_t* buf, uint32_t cap) = 0;
};

// I2C 連線參數（實體驅動填入；腳位為 placeholder，等腳位表）
struct CamI2cConfig {
  uint32_t clockHz = 400000;  // 預設 400 kHz，可調
  // 腳位 placeholder：數值 0xFF 表示未定，禁止當成真實腳位使用
  static constexpr uint8_t kPinSdaPlaceholder = 0xFF;
  static constexpr uint8_t kPinSclPlaceholder = 0xFF;
  uint8_t pinSda = kPinSdaPlaceholder;
  uint8_t pinScl = kPinSclPlaceholder;
  uint8_t addr7 = 0x00;  // 裝置 7-bit 位址；0 = 未定，等硬體定案
};

}  // namespace Components
#endif
