// Dht22SimIo：DHT22 波形模擬（給 UsDht22Sim 與 host 測試）
#ifndef ENVSENSING_DHT22SIMIO_HPP
#define ENVSENSING_DHT22SIMIO_HPP
#include "Components/EnvSensing/UsDht22/Dht22Core.hpp"
namespace Components {
enum class Dht22SimFault : uint8_t { NONE = 0, NO_RESPONSE = 1, BAD_CHECKSUM = 2, STUCK = 3, OUT_OF_RANGE = 4, SHORT_FRAME = 5 };
class Dht22SimIo : public IDht22Io {
 public:
  void setTruth(float temp_C, float hum_pct) { m_t = temp_C; m_h = hum_pct; }
  void setFault(Dht22SimFault f) { m_fault = f; }
  bool configure() override { return true; }
  bool start() override;
  int fetchHighs(uint16_t* highs_us, int maxCount) override;
  // 依 DHT22 協定編碼 5 bytes（測試也用）
  static void encode(float temp_C, float hum_pct, uint8_t b[5]);
 private:
  float m_t = 25.0f, m_h = 50.0f;
  Dht22SimFault m_fault = Dht22SimFault::NONE;
  bool m_started = false;
  uint8_t m_frozen[5] = {0, 0, 0, 0, 0};
  bool m_haveFrozen = false;
  unsigned m_n = 0;
};
}  // namespace Components
#endif
