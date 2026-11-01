// ======================================================================
// \title  SepServoDriver.hpp
// \author 伺服馬達驅動組（Evgeny）
// \brief  MG996R SEP_1–4；once-only via NvFlags placeholder
// Base ID 0x3100
// ======================================================================
#ifndef COMPONENTS_SEPSERVODRIVER_HPP
#define COMPONENTS_SEPSERVODRIVER_HPP

#include "Components/Servo/Pwm/ServoPwmHal.hpp"
#include "Components/Servo/SepServoDriver/SepServoDriverComponentAc.hpp"

namespace Components {

class SepServoDriver final : public SepServoDriverComponentBase {
 public:
  explicit SepServoDriver(const char* compName);
  ~SepServoDriver() override = default;

  void configure(const U8* gpioPins, U8 nPins);
  void configure(const ServoPwmChannel* ch, U8 n, U8 ledcTimerId);

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void cmdIn_handler(FwIndexType portNum, const RocketTypes::SepCmd& c) override;
  void uvIn_handler(FwIndexType portNum, const RocketTypes::UvState& st) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();
  F64 now_s() const;
  void enterState(RocketTypes::DrvState::T st, U8 faultReason = 0);
  void applyPwm();
  void emitPwmOut();
  void writeTlm();
  U16 clampUs(U16 raw) const;
  U8 readFiredMask();
  bool tryFireChannel(U8 ch);
  void ensurePwmConfigured();

  RocketTypes::DrvState::T m_state;
  RocketTypes::UvState m_uv;
  U32 m_lastSeq;
  bool m_haveSeq;
  U16 m_out_us[4];
  U8 m_activeMask;   // currently commanding fire pulse
  U8 m_lockedMask;   // channels held at lock (unreleased or fault hold)
  F64 m_fireStart_s[4];
  U32 m_tlmDiv;
  F64 m_stateEnter_s;
  bool m_pwmConfigured;
  U8 m_ledcTimerId;
  ServoPwmChannel m_chCfg[ServoPwmHal::kMaxChannels];
  U8 m_gpioN;
  ServoPwmHal m_pwm;

  U16 m_lock_us;
  U16 m_fire_us;
  F32 m_fireHold_s;
  U16 m_min_us;
  U16 m_max_us;
  F32 m_selftestDeadline_s;
};

}  // namespace Components
#endif
