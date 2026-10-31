// Dht22Core：US DHT22 驅動核心（與 F´ 無關，可在 host 測試）
// 時序：rg1Hz，偶數 tick 觸發（RMT 送起始脈衝並開始擷取），下一個 tick 解碼 → 每 2 s 一筆。
// 不 sleep、不讀系統時間；ISR 只搬資料（見 Dht22RmtEsp32.cpp），解碼在 rg1Hz handler。
#ifndef ENVSENSING_DHT22CORE_HPP
#define ENVSENSING_DHT22CORE_HPP
#include <cstdint>

namespace Components {

enum class DhtState : uint8_t { OFF = 0, INIT = 1, SELFTEST = 2, READY = 3, FAULT = 4 };

enum class DhtFault : uint8_t {
  NO_RESPONSE = 1,  // 連續 errMax 次沒有擷取到波形
  CHECKSUM = 2,     // 連續 errMax 次 checksum 錯或位元數不足
  OUT_OF_RANGE = 3, // 連續 errMax 次超出 DHT22 量測範圍
  STUCK = 4,        // 原始 4 bytes 連續 stuckN 筆相同（預設關閉）
  IO_INIT = 5,      // RMT 設定失敗
};

struct HumTempOut {
  double t_s;
  uint32_t seq;
  bool valid;
  float temp_K;
  float humidity_frac;
};

// 平台 I/O（ESP32 用 RMT 實作，Sim 用 Dht22SimIo）
class IDht22Io {
 public:
  virtual ~IDht22Io() = default;
  virtual bool configure() = 0;
  // 送起始脈衝並開始擷取；非阻塞
  virtual bool start() = 0;
  // 取回擷取到的「高電位」持續時間（µs，依序）；尚未完成或無訊號回傳 -1
  virtual int fetchHighs(uint16_t* highs_us, int maxCount) = 0;
};

class IDht22Sink {
 public:
  virtual ~IDht22Sink() = default;
  virtual void onSample(const HumTempOut& s) = 0;
  virtual void onStateChange(DhtState from, DhtState to) = 0;
  virtual void onFault(DhtFault code, uint32_t detail) = 0;
};

struct Dht22Config {
  uint8_t errMax = 3;
  uint16_t stuckN = 0;          // 0 = 關閉（DHT22 解析度 0.1，靜置時合法地長時間同值）
  uint8_t selftestN = 3;
  uint16_t faultRetryTicks = 10;  // rg1Hz tick
  uint8_t maxRetries = 5;
  uint16_t bitThreshold_us = 48;  // '0' ≈ 26–28 µs，'1' ≈ 70 µs
};

namespace Dht22Decode {
// 從高電位序列取最後 40 個位元；成功回傳 true 並填 5 bytes
bool bitsFromHighs(const uint16_t* highs, int n, uint16_t thr_us, uint8_t out[5]);
bool checksumOk(const uint8_t b[5]);
void toPhysical(const uint8_t b[5], float& temp_C, float& hum_pct);
}  // namespace Dht22Decode

class Dht22Core {
 public:
  Dht22Core(IDht22Io& io, IDht22Sink& sink);
  void setConfig(const Dht22Config& c) { m_cfg = c; }
  void tick(double t_s);  // 每個 rg1Hz tick 呼叫
  DhtState state() const { return m_state; }
  uint32_t errorCount() const { return m_totalErrors; }
  uint32_t seq() const { return m_seq; }

 private:
  void setState(DhtState s);
  void fault(DhtFault c, uint32_t d);
  void decode();
  bool countErr(uint8_t& ctr, DhtFault c, uint32_t d);

  IDht22Io& m_io;
  IDht22Sink& m_sink;
  Dht22Config m_cfg;
  DhtState m_state = DhtState::OFF;
  bool m_armed = false;
  double m_trigT = 0;
  uint8_t m_errNoResp = 0, m_errCrc = 0, m_errRange = 0;
  uint16_t m_good = 0;
  uint16_t m_stuck = 0;
  uint32_t m_lastRaw = 0xFFFFFFFF;
  uint16_t m_faultTicks = 0;
  uint8_t m_retries = 0;
  uint32_t m_totalErrors = 0;
  uint32_t m_seq = 0;
};

}  // namespace Components
#endif
