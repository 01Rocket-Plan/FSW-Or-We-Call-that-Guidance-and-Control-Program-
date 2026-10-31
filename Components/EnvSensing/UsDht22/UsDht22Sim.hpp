// UsDht22Sim：同一個 Dht22Core 接波形模擬器
#ifndef ENVSENSING_USDHT22SIM_HPP
#define ENVSENSING_USDHT22SIM_HPP

#include "Components/EnvSensing/UsDht22/UsDht22SimComponentAc.hpp"
#include "Components/EnvSensing/UsDht22/Dht22Core.hpp"
#include "Components/EnvSensing/UsDht22/Dht22SimIo.hpp"

namespace Components {

class UsDht22Sim final : public UsDht22SimComponentBase, private IDht22Sink {
 public:
  explicit UsDht22Sim(const char* compName);
  ~UsDht22Sim() override = default;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void SIM_SET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 temp_C, F32 humidity_pct) override;
  void SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Components::DhtSimFault& fault) override;
  void onSample(const HumTempOut& s) override;
  void onStateChange(DhtState from, DhtState to) override;
  void onFault(DhtFault code, uint32_t detail) override;

  Dht22SimIo m_io;
  Dht22Core m_core;
};

}  // namespace Components
#endif
