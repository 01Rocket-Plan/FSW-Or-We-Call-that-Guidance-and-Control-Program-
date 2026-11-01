// Dht22RmtEsp32：ESP32（ESP-IDF v5 新版 RMT 驅動）上的 DHT22 I/O
// 起始低脈衝由 RMT TX（開汲極、loop back）硬體產生，回應由 RMT RX 硬體擷取，CPU 不等待。
// ISR（on_recv_done）只記錄符號數與完成旗標；解碼在 rg1Hz handler（Dht22Core）。
// 只在 ESP_PLATFORM 下編譯；尚未上實機驗證，腳位與時脈請 MCU 組確認。
#ifndef ENVSENSING_DHT22RMTESP32_HPP
#define ENVSENSING_DHT22RMTESP32_HPP
#include "Components/EnvSensing/UsDht22/Dht22Core.hpp"

#ifdef ESP_PLATFORM
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

namespace Components {

class Dht22RmtEsp32 : public IDht22Io {
 public:
  explicit Dht22RmtEsp32(int gpioNum) : m_gpio(gpioNum) {}
  bool configure() override;
  bool start() override;
  int fetchHighs(uint16_t* highs_us, int maxCount) override;

 private:
  static bool onRecvDone(rmt_channel_handle_t ch, const rmt_rx_done_event_data_t* ed, void* ctx);
  int m_gpio;
  rmt_channel_handle_t m_tx = nullptr;
  rmt_channel_handle_t m_rx = nullptr;
  rmt_encoder_handle_t m_enc = nullptr;
  rmt_symbol_word_t m_sym[64];
  volatile size_t m_nSym = 0;
  volatile bool m_done = false;
  bool m_configured = false;
};

}  // namespace Components
#endif  // ESP_PLATFORM
#endif
