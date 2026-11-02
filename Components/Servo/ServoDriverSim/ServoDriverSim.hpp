// ServoDriverSim：ServoDriver 的 Sim 版（埠集合同真驅動）。核心見 Components/Servo/SimCore。
// Base ID：servoDriverSim 0x3080、gimbalServoDriverSim 0x3280（channel_count=2）
#ifndef COMPONENTS_SERVODRIVERSIM_HPP
#define COMPONENTS_SERVODRIVERSIM_HPP
#include "Components/Servo/ServoDriverSim/ServoDriverSimComponentAc.hpp"
#include "Components/Servo/SimCore/ServoSimCore.hpp"

namespace Components {

class ServoDriverSim final : public ServoDriverSimComponentBase, private ServoSim::ISimHw, private ServoSim::IServoSink {
 public:
  explicit ServoDriverSim(const char* compName);
  ~ServoDriverSim() override = default;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void cmdIn_handler(FwIndexType portNum, const RocketTypes::ServoCmd& c) override;
  void uvIn_handler(FwIndexType portNum, const RocketTypes::UvState& st) override;
  void parametersLoaded() override;

  bool selfTestDone(bool& fail) override { fail = false; return true; }  // Sim：自測直接通過
  void pwmOut(uint8_t ch, uint16_t used_us) override;
  void evEnterReady() override;
  void evEnterFault(ServoSim::FaultReason r) override;
  void evCmdDropped(uint32_t seq, ServoSim::DropReason r) override;
  void evPulseClamped(uint8_t ch, uint16_t raw, uint16_t used) override;
  void evCmdTimeout(float age_s) override;
  void evLockNeutral(ServoSim::NeutralSource s) override;
  void evUvSeen(ServoSim::UvState st) override;

  void writeTlm();
  F64 now_s();

  ServoSim::ServoSimCore m_core;
  U16 m_out_us[4];
  U32 m_tick;
  U32 m_periodTicks;
  bool m_started;
  bool m_lockNeutral;
};

}  // namespace Components
#endif
