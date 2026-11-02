// SepServoDriverSim：SepServoDriver 的 Sim 版（埠集合同真驅動）。Base ID 0x3180。核心見 Components/Servo/SimCore。
#ifndef COMPONENTS_SEPSERVODRIVERSIM_HPP
#define COMPONENTS_SEPSERVODRIVERSIM_HPP
#include "Components/Servo/SepServoDriverSim/SepServoDriverSimComponentAc.hpp"
#include "Components/Servo/SimCore/ServoSimCore.hpp"

namespace Components {

class SepServoDriverSim final : public SepServoDriverSimComponentBase,
                                private ServoSim::ISimHw, private ServoSim::INvFlags, private ServoSim::ISepSink {
 public:
  explicit SepServoDriverSim(const char* compName);
  ~SepServoDriverSim() override = default;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void cmdIn_handler(FwIndexType portNum, const RocketTypes::SepCmd& c) override;
  void uvIn_handler(FwIndexType portNum, const RocketTypes::UvState& st) override;
  void parametersLoaded() override;

  bool selfTestDone(bool& fail) override { fail = false; return true; }
  bool sepReleasedGet(uint8_t& mask) override;   // 走 sepReleasedGet 輸出埠（NvFlags 或其 mock）
  bool sepReleasedSet(uint8_t mask) override;    // 走 sepReleasedSet 輸出埠
  void pwmOut(uint8_t ch, uint16_t used_us) override;
  void evEnterReady() override;
  void evEnterFault(ServoSim::FaultReason r) override;
  void evCmdDropped(uint32_t seq, ServoSim::DropReason r) override;
  void evSepFire(uint8_t ch) override;
  void evSepAlreadyFired(uint8_t ch) override;
  void evSepFaultHold(uint8_t mask) override;
  void evSepNvWriteFail(uint8_t ch) override;
  void evUvSeen(ServoSim::UvState st) override;

  F64 now_s();

  ServoSim::SepSimCore m_core;
  U16 m_out_us[4];
  U32 m_tick;
  bool m_started;
};

}  // namespace Components
#endif
