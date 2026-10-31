#include "Components/EnvSensing/UsDht22/Dht22RmtEsp32.hpp"
#ifdef ESP_PLATFORM
#include "esp_attr.h"

namespace EnvSensing {

static constexpr uint32_t RES_HZ = 1000000;  // 1 tick = 1 µs
static constexpr uint16_t START_LOW_US = 1100;  // DHT22 要求主機拉低 ≥ 1 ms

bool IRAM_ATTR Dht22RmtEsp32::onRecvDone(rmt_channel_handle_t, const rmt_rx_done_event_data_t* ed, void* ctx) {
  auto* self = static_cast<Dht22RmtEsp32*>(ctx);
  self->m_nSym = ed->num_symbols;  // 符號已由硬體寫入 m_sym，ISR 只記數
  self->m_done = true;
  return false;
}

bool Dht22RmtEsp32::configure() {
  if (m_configured) return true;
  rmt_tx_channel_config_t txc = {};
  txc.gpio_num = static_cast<gpio_num_t>(m_gpio);
  txc.clk_src = RMT_CLK_SRC_DEFAULT;
  txc.resolution_hz = RES_HZ;
  txc.mem_block_symbols = 48;
  txc.trans_queue_depth = 1;
  txc.flags.io_od_mode = 1;    // 開汲極，外部上拉
  txc.flags.io_loop_back = 1;  // 同一腳位給 RX
  if (rmt_new_tx_channel(&txc, &m_tx) != ESP_OK) return false;

  rmt_rx_channel_config_t rxc = {};
  rxc.gpio_num = static_cast<gpio_num_t>(m_gpio);
  rxc.clk_src = RMT_CLK_SRC_DEFAULT;
  rxc.resolution_hz = RES_HZ;
  rxc.mem_block_symbols = 64;
  if (rmt_new_rx_channel(&rxc, &m_rx) != ESP_OK) return false;

  rmt_copy_encoder_config_t ec = {};
  if (rmt_new_copy_encoder(&ec, &m_enc) != ESP_OK) return false;

  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = &Dht22RmtEsp32::onRecvDone;
  if (rmt_rx_register_event_callbacks(m_rx, &cbs, this) != ESP_OK) return false;
  if (rmt_enable(m_tx) != ESP_OK || rmt_enable(m_rx) != ESP_OK) return false;
  m_configured = true;
  return true;
}

bool Dht22RmtEsp32::start() {
  if (!m_configured) return false;
  m_done = false;
  m_nSym = 0;
  rmt_receive_config_t rc = {};
  rc.signal_range_min_ns = 2000;       // 濾掉 < 2 µs 突波
  rc.signal_range_max_ns = 1500000;    // 1.5 ms 無變化視為結束（> 起始低脈衝 1.1 ms）
  if (rmt_receive(m_rx, m_sym, sizeof(m_sym), &rc) != ESP_OK) return false;
  rmt_symbol_word_t pulse = {};
  pulse.level0 = 0;
  pulse.duration0 = START_LOW_US;
  pulse.level1 = 1;
  pulse.duration1 = 10;
  rmt_transmit_config_t tc = {};
  tc.loop_count = 0;
  tc.flags.eot_level = 1;  // 結束後釋放（高）
  return rmt_transmit(m_tx, m_enc, &pulse, sizeof(pulse), &tc) == ESP_OK;
}

int Dht22RmtEsp32::fetchHighs(uint16_t* out, int max) {
  if (!m_done) return -1;
  int n = 0;
  const size_t ns = m_nSym;
  for (size_t i = 0; i < ns && i < 64; i++) {
    if (m_sym[i].level0 == 1 && m_sym[i].duration0 > 0 && n < max) out[n++] = m_sym[i].duration0;
    if (m_sym[i].level1 == 1 && m_sym[i].duration1 > 0 && n < max) out[n++] = m_sym[i].duration1;
  }
  m_done = false;
  return n;
}

}  // namespace EnvSensing
#endif  // ESP_PLATFORM
