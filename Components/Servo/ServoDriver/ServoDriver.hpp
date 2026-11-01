// ======================================================================
// \title  ServoDriver.hpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  SG90 PWM 驅動（尾翼／雲台實例）；握手 + 50 Hz 零階保持
// Base ID 0x3000（雲台實例 0x3200）
// ======================================================================
#ifndef COMPONENTS_SERVODRIVER_HPP
#define COMPONENTS_SERVODRIVER_HPP

#include "Components/Servo/Pwm/ServoPwmHal.hpp"
#include "Components/Servo/ServoDriver/ServoDriverComponentAc.hpp"

namespace Components {

class ServoDriver final : public ServoDriverComponentBase {
 public:
  explicit ServoDriver(const char* compName);
  ~ServoDriver() override = default;

  //! GPIO／LEDC 通道表（拓樸呼叫）；未呼叫則用 ServoPwmPins 預設（可能仍為未指派）
  void configure(const U8* gpioPins, U8 nPins);
  void configure(const ServoPwmChannel* ch, U8 n, U8 ledcTimerId);

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void cmdIn_handler(FwIndexType portNum, const RocketTypes::ServoCmd& c) override;
  void uvIn_handler(FwIndexType portNum, const RocketTypes::UvState& st) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  F64 now_s() const;
  void enterState(RocketTypes::DrvState::T st, U8 faultReason = 0);
  void applyOutputs(bool emitTlm);
  U16 clampPulse(U8 ch, U16 raw);
  void emitPwmOut();
  void writeTlm();
  bool pwmSelftestOk() const;
  void ensurePwmConfigured();

  RocketTypes::DrvState::T m_state;
  RocketTypes::UvState m_uv;
  U32 m_lastSeq;
  bool m_haveSeq;
  F64 m_lastCmd_s;
  bool m_haveCmd;
  bool m_lockNeutral;
  U16 m_out_us[4];
  U32 m_tick;
  U32 m_pwmPeriodTicks;
  U32 m_tlmDiv;
  U32 m_clampCount;
  U32 m_dropCount;
  F64 m_stateEnter_s;
  bool m_pwmConfigured;
  U8 m_ledcTimerId;
  ServoPwmChannel m_chCfg[ServoPwmHal::kMaxChannels];
  U8 m_gpioN;
  ServoPwmHal m_pwm;

  U16 m_neutral_us;
  U16 m_min_us;
  U16 m_max_us;
  F32 m_cmdTimeout_s;
  U16 m_pwmHz;
  U8 m_channelCount;
  F32 m_selftestDeadline_s;
};

}  // namespace Components
#endif
