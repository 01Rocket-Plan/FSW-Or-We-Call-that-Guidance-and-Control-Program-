// ======================================================================
// \title  PadController.cpp
// \brief  發射架主控 F´ 外殼：ioGet → Core.tick → actOut；臍帶／NRF24 位元組 → Core；10 Hz 狀態幀＋遙測。
// ======================================================================
#include "Components/Wireless/PadController/PadController.hpp"

namespace Components {

namespace {

PadState toFpp(PadLink::PadState s) { return static_cast<PadState::T>(static_cast<U8>(s)); }
PadReason toFpp(PadLink::PadReason r) { return static_cast<PadReason::T>(static_cast<U8>(r)); }
PadCmdOp toFpp(PadLink::PadCmdOp o) { return static_cast<PadCmdOp::T>(static_cast<U8>(o)); }
PadCmdResult toFpp(PadLink::PadCmdResult r) { return static_cast<PadCmdResult::T>(static_cast<U8>(r)); }

//! 驗證器還沒接 → bootId 先固定 0（接 HMAC 時一起改成開機產生）
constexpr std::uint8_t kBootId = 0;

}  // namespace

// 驗證器先不接（nullptr）→ deny-by-default，見 .hpp 檔頭
PadController::PadController(const char* compName)
    : PadControllerComponentBase(compName), m_core(nullptr, kBootId, this), m_tickDiv(0), m_paramsLoaded(false), m_radioBuf{}, m_umbBuf{} {}

F64 PadController::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

// F´ 開機載參數時會逐個呼叫 parameterUpdated（其他參數還沒載完）→ 那時不套，
// 等 parametersLoaded 一次套全部；之後 GDS 改單一參數才即時套。
void PadController::parameterUpdated(FwPrmIdType) {
  if (m_paramsLoaded) {
    loadParams();
  }
}
void PadController::parametersLoaded() {
  m_paramsLoaded = true;
  loadParams();
}

void PadController::loadParams() {
  // prmDb 沒存的參數回傳 FPP 預設值（與 PadParams 預設一致）
  Fw::ParamValid v;
  PadLink::PadParams p;
  p.countdown_s = this->paramGet_countdown_s(v);
  p.thrustConfirm_s = this->paramGet_thrustConfirm_s(v);
  p.releaseMargin_s = this->paramGet_releaseMargin_s(v);
  p.clampActuatorLatency_s = this->paramGet_clampActuatorLatency_s(v);
  p.liftoffTimeout_s = this->paramGet_liftoffTimeout_s(v);
  p.releaseDeadline_s = this->paramGet_releaseDeadline_s(v);
  p.liftoffLoadMin_N = this->paramGet_liftoffLoadMin_N(v);
  p.liftoffLoadHold_s = this->paramGet_liftoffLoadHold_s(v);
  p.clampOpenTimeout_s = this->paramGet_clampOpenTimeout_s(v);
  p.groundLinkTimeout_s = this->paramGet_groundLinkTimeout_s(v);
  p.umbilicalTimeout_s = this->paramGet_umbilicalTimeout_s(v);
  p.maxTilt_rad = this->paramGet_maxTilt_rad(v);
  p.padVmin_V = this->paramGet_padVmin_V(v);
  (void)m_core.configure(p);  // 不合理 → onBadParams 事件，之後 ARM 以 BAD_PARAMS 拒絕
}

void PadController::schedIn_handler(FwIndexType, U32) {
  PadLink::PadInputs in;  // 沒接 ioGet（SIL）→ 全 false／0 → 互鎖全紅，只能停在 SAFE
  if (this->isConnected_ioGet_OutputPort(0)) {
    const PadIoSample s = this->ioGet_out(0);
    in.seated = s.get_seated();
    in.continuity = s.get_continuity();
    in.keyOn = s.get_keyOn();
    in.railJam = s.get_railJam();
    in.offRail = s.get_offRail();
    in.thrustOk = s.get_thrustOk();
    in.clampOpen = s.get_clampOpen();
    in.load_N = static_cast<double>(s.get_load_N());
    in.tilt_rad = static_cast<double>(s.get_tilt_rad());
    in.padV_V = static_cast<double>(s.get_padV_V());
  }
  const PadLink::PadOutputs o = m_core.tick(in, nowSeconds());
  if (this->isConnected_actOut_OutputPort(0)) {
    this->actOut_out(0, PadActCmd(o.igniterOn, o.clampRelease));
  }
  if (++m_tickDiv >= kStatusEveryTicks) {
    m_tickDiv = 0;
    sendFrames();
    writeTlm();
  }
}

void PadController::umbilicalIn_handler(FwIndexType, Fw::Buffer& fwBuffer) {
  m_core.onUmbilicalBytes(fwBuffer.getData(), static_cast<std::size_t>(fwBuffer.getSize()), nowSeconds());
}

void PadController::radioIn_handler(FwIndexType, Fw::Buffer& fwBuffer) {
  (void)m_core.onGroundPacket(fwBuffer.getData(), static_cast<std::size_t>(fwBuffer.getSize()), nowSeconds());
}

void PadController::sendFrames() {
  if (this->isConnected_radioOut_OutputPort(0)) {
    const std::size_t n = m_core.buildStatusFrame(m_radioBuf, sizeof(m_radioBuf));
    if (n > 0) {
      Fw::Buffer b(m_radioBuf, static_cast<Fw::Buffer::SizeType>(n));
      this->radioOut_out(0, b);
    }
  }
  if (this->isConnected_umbilicalOut_OutputPort(0)) {
    const std::size_t n = m_core.buildUmbilicalFrame(m_umbBuf, sizeof(m_umbBuf));
    if (n > 0) {
      Fw::Buffer b(m_umbBuf, static_cast<Fw::Buffer::SizeType>(n));
      this->umbilicalOut_out(0, b);
    }
  }
}

void PadController::writeTlm() {
  this->tlmWrite_State(toFpp(m_core.state()));
  this->tlmWrite_LastReason(toFpp(m_core.lastReason()));
  this->tlmWrite_Interlocks(m_core.interlocks());
  this->tlmWrite_CountdownRemaining_s(static_cast<F32>(m_core.countdownRemaining_s()));
  this->tlmWrite_MissionTime_s(m_core.missionTime_s());
  this->tlmWrite_RocketMode(m_core.lastRocketStatus().flightMode);
  this->tlmWrite_RocketBit(m_core.lastRocketStatus().bitFlags);
  this->tlmWrite_UmbilicalCrcErrors(m_core.umbilicalCrcErrors());
  this->tlmWrite_GroundRejects(m_core.groundRejects());
}

void PadController::onStateChange(PadLink::PadState from, PadLink::PadState to, PadLink::PadReason reason) {
  this->log_ACTIVITY_HI_PadStateChange(toFpp(from), toFpp(to), toFpp(reason));
  switch (to) {
    case PadLink::PadState::HANG_FIRE:
      this->log_WARNING_HI_PadHangFire();
      break;
    case PadLink::PadState::HOLD_ABORT:
      this->log_WARNING_HI_PadHoldAbort(toFpp(reason));
      break;
    case PadLink::PadState::NO_LIFTOFF:
      if (reason == PadLink::PadReason::CLAMP_STUCK) {
        this->log_WARNING_HI_PadClampStuck();
      } else {
        this->log_WARNING_HI_PadNoLiftoff();
      }
      break;
    default:
      break;
  }
}

void PadController::onCommand(PadLink::PadCmdOp op, PadLink::PadCmdResult result, PadLink::PadReason reason) {
  this->log_ACTIVITY_LO_PadCommand(toFpp(op), toFpp(result), toFpp(reason));
}

void PadController::onBadParams(const PadLink::PadParams& p) {
  this->log_WARNING_HI_PadBadParams(p.thrustConfirm_s, p.releaseMargin_s, p.clampActuatorLatency_s,
                                    p.releaseDeadline_s);
}

}  // namespace Components
