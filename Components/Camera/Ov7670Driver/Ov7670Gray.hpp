// Ov7670Gray.hpp：從 OV7670 的 YUV422（YUYV）幀取出 Y（亮度）＝灰階。
// YUYV 順序（TSLB[3]＝0、COM13[0]＝0）：Y0 U0 Y1 V0 Y2 U1 …，Y 在偶數位元組。
// 不配置記憶體：呼叫端給目的緩衝（驅動用自己的 2 KB 分塊緩衝），一次只轉一塊。
#ifndef COMPONENTS_CAMERA_OV7670DRIVER_OV7670GRAY_HPP
#define COMPONENTS_CAMERA_OV7670DRIVER_OV7670GRAY_HPP

#include <cstdint>

namespace Components {

//! YUYV 長度 rawLen 位元組裡有幾個灰階像素
inline uint32_t ov7670GrayPixels(uint32_t rawLen) { return rawLen / 2u; }

//! 從第 firstPixel 個像素開始，取最多 n 個 Y 寫進 dst；回傳實際寫入的像素數。
inline uint32_t ov7670PackY(const uint8_t* yuyv, uint32_t rawLen, uint32_t firstPixel,
                            uint8_t* dst, uint32_t n) {
  if (yuyv == nullptr || dst == nullptr) return 0;
  const uint32_t total = ov7670GrayPixels(rawLen);
  if (firstPixel >= total) return 0;
  if (n > total - firstPixel) n = total - firstPixel;
  for (uint32_t i = 0; i < n; ++i) {
    dst[i] = yuyv[2u * (firstPixel + i)];
  }
  return n;
}

}  // namespace Components

#endif
