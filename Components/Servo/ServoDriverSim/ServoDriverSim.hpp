// ServoDriverSim — Sim shell for ServoDriver. Core: SimCore + Sg90SimDevice. Bases 0x3080 / 0x3280.
#ifndef COMPONENTS_SERVODRIVERSIM_HPP
#define COMPONENTS_SERVODRIVERSIM_HPP
#include "Components/Servo/ServoDriverSim/ServoDriverSimComponentAc.hpp"
#include "Components/Servo/ModelCores/Sg90SimDevice.hpp"
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
  void evModelRoleMismatch(ServoSim::ServoModel model, ServoSim::ServoRole role, uint8_t channel_count) override;

  void writeTlm();
  F64 now_s();

  ServoSim::ServoSimCore m_core;
  Sg90SimDevice m_sg90Dev;  // SIL shaft-angle slew (datasheet-nominal)
  U16 m_out_us[4];
  U32 m_tick;
  U32 m_periodTicks;
  bool m_started;
  bool m_lockNeutral;
  Components::ServoModel m_model;
  Components::ServoRole m_role;
};

}  // namespace Components
#endif
