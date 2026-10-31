// UsDht22：DHT22 的 F´ 元件外殼（邏輯在 Dht22Core；平台 I/O 由 configure() 注入）
#ifndef ENVSENSING_USDHT22_HPP
#define ENVSENSING_USDHT22_HPP

#include "Components/EnvSensing/UsDht22/UsDht22ComponentAc.hpp"
#include "Components/EnvSensing/UsDht22/Dht22Core.hpp"

namespace EnvSensing {

class UsDht22 final : public UsDht22ComponentBase, private IDht22Io, private IDht22Sink {
 public:
  explicit UsDht22(const char* compName);
  ~UsDht22() override = default;
  //! 拓樸設定：注入平台 I/O（ESP32 用 Dht22RmtEsp32）
  void configure(IDht22Io& io);

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  bool configure() override;
  bool start() override;
  int fetchHighs(uint16_t* highs_us, int maxCount) override;
  void onSample(const HumTempOut& s) override;
  void onStateChange(DhtState from, DhtState to) override;
  void onFault(DhtFault code, uint32_t detail) override;

  IDht22Io* m_io;
  Dht22Core m_core;
};

}  // namespace EnvSensing
#endif
