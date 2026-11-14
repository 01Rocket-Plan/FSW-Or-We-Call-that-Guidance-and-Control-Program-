// ======================================================================
// \title  FlightMode.cpp
// \author 殼：MCU 組（EdwardWang+Claude）｜邏輯：FlightModeCore（照 SSS，Edward 2026-11-14 授權）
// \brief  F´ 外殼：搬輸入 → core.tick() → 廣播模式、送 SepCmd、寫 NV 閂鎖、事件。
//         這裡只有「搬資料」與「把 core 的輸出送出去」，沒有任何轉移判斷。
// ======================================================================
#include "Components/MCU/FlightMode/FlightMode.hpp"

#include <cmath>

namespace Mcu {

namespace {
constexpr U32 kTlmDivider = 100;  // 1 kHz → 10 Hz

// FmPad 數值必須等於 Components.PadState（PadLink.fpp）
static_assert(static_cast<U8>(Components::PadState::SAFE) == FmPad::SAFE, "PadState");
static_assert(static_cast<U8>(Components::PadState::ARMED) == FmPad::ARMED, "PadState");
static_assert(static_cast<U8>(Components::PadState::COUNTDOWN) == FmPad::COUNTDOWN, "PadState");
static_assert(static_cast<U8>(Components::PadState::IGNITION) == FmPad::IGNITION, "PadState");
static_assert(static_cast<U8>(Components::PadState::RELEASED) == FmPad::RELEASED, "PadState");
static_assert(static_cast<U8>(Components::PadState::LIFTED_OFF) == FmPad::LIFTED_OFF, "PadState");
static_assert(static_cast<U8>(Components::PadState::HANG_FIRE) == FmPad::HANG_FIRE, "PadState");
static_assert(static_cast<U8>(Components::PadState::HOLD_ABORT) == FmPad::HOLD_ABORT, "PadState");
static_assert(static_cast<U8>(Components::PadState::NO_LIFTOFF) == FmPad::NO_LIFTOFF, "PadState");

RocketTypes::FlightModeId toFpp(FmMode m) {
  return RocketTypes::FlightModeId(static_cast<RocketTypes::FlightModeId::T>(static_cast<U8>(m)));
}
}  // namespace

FlightMode::FlightMode(const char* compName)
    : FlightModeComponentBase(compName),
      m_deployRole(FmRole::DS),
      m_lastMode(RocketTypes::FlightModeId::PAD_IDLE),
      m_sepSeq(0),
      m_chuteSeq(0),
      m_tlmDiv(0),
      m_warnedIncomplete(false) {}

FlightMode::~FlightMode() {}

F64 FlightMode::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void FlightMode::applyRole(FmRole role) {
  if (role == m_core.role()) return;
  m_core.setRole(role);
  m_lastMode = toFpp(role == FmRole::US ? m_core.state().us : m_core.state().ds);  // 不發假的 ModeChanged
}

void FlightMode::setDeploymentRole(FmRole role) {
  m_deployRole = role;
  applyRole(role);
}

void FlightMode::loadParams() {
  Fw::ParamValid v;
  const FmRoleId r = this->paramGet_role(v);
  // prmDb 有存 role（VALID）→ 用 prmDb；否則用拓樸指定的部署角色
  const FmRole role = (v == Fw::ParamValid::VALID) ? ((r == FmRoleId::US) ? FmRole::US : FmRole::DS) : m_deployRole;
  applyRole(role);
  // [FM-P] 門檻參數 → FmParams（Edward 加 param 後在這裡搬）
}

void FlightMode::parameterUpdated(FwPrmIdType) { loadParams(); }
void FlightMode::parametersLoaded() { loadParams(); }

void FlightMode::stateIn_handler(FwIndexType, const RocketTypes::StateOut& s) {
  m_in.haveState = true;
  m_in.accel_u_mps2 = s.get_accAxial_mps2();
  const F32 ve = s.get_vel_enu_mps()[0], vn = s.get_vel_enu_mps()[1], vu = s.get_vel_enu_mps()[2];
  m_in.v_est_mps = std::sqrt(ve * ve + vn * vn + vu * vu);
  m_in.vU_mps = vu;
  m_in.tilt_rad = s.get_tilt_rad();
  // 機體橫向角速度（俯仰 ωy、偏航 ωz，不含滾轉 ωx；conventions §2 業界標準）
  m_in.rate_rads = std::hypot(s.get_rate_body_rads()[1], s.get_rate_body_rads()[2]);
  m_in.imu_fresh = s.get_imuValid();
  m_in.velValid = s.get_velValid();
  m_in.alt_m = s.get_alt_m();
}

void FlightMode::uvIn_handler(FwIndexType, const RocketTypes::UvState& st) {
  m_in.uvState = static_cast<uint8_t>(st.e);
}

// [FM-IN-2／3] 只搬資料。無效樣本不更新（保持上一筆；開機預設 false）。
void FlightMode::discreteIn_handler(FwIndexType, const RocketTypes::DiscreteSample& s) {
  if (!s.get_hdr().get_valid()) return;
  m_in.sep_switch_open = s.get_sepSwitchOpen();
  if (m_core.role() == FmRole::US) {
    m_in.contact_us = s.get_groundContact();
  } else {
    m_in.contact_ds = s.get_groundContact();
  }
}

// [FM-03] 只搬資料。linkOk＝UmbilicalLink 判定架狀態夠新（padLinkTimeout_s）。
void FlightMode::padStatusIn_handler(FwIndexType, const Components::PadState& padState, F32 countdown_s, U8 flags,
                                     bool linkOk) {
  m_in.padLinkOk = linkOk;
  m_in.padState = static_cast<uint8_t>(padState.e);
  m_in.padCountdown_s = countdown_s;
  m_in.padFlags = flags;
}

void FlightMode::emitOutputs(const FmOutputs& o) {
  const RocketTypes::FlightModeId mode = toFpp(o.mode);
  if (mode != m_lastMode) {
    this->log_ACTIVITY_HI_ModeChanged(m_lastMode, mode);
    m_lastMode = mode;
  }
  for (FwIndexType i = 0; i < this->getNum_modeOut_OutputPorts(); ++i) {
    if (this->isConnected_modeOut_OutputPort(i)) this->modeOut_out(i, mode);
  }

  if (o.sepFire) {
    this->log_ACTIVITY_HI_SepCommanded(o.sepMask);
    if (this->isConnected_sepCommandedOut_OutputPort(0)) (void)this->sepCommandedOut_out(0);
    if (this->isConnected_sepCmdOut_OutputPort(0)) {
      this->sepCmdOut_out(0, RocketTypes::SepCmd(this->nowSeconds(), ++m_sepSeq, o.sepMask, true));
    }
  }
  if (o.chuteDs) {
    this->log_ACTIVITY_HI_ChuteDsCommanded();
    if (this->isConnected_dsChuteCommandedOut_OutputPort(0)) (void)this->dsChuteCommandedOut_out(0);
    if (this->isConnected_chuteCmdOut_OutputPort(0)) {
      // side＝SEP_FAIL 側傘
      this->chuteCmdOut_out(0, RocketTypes::ChuteCmd(this->nowSeconds(), ++m_chuteSeq, RocketTypes::Stage::DS,
                                                     o.chuteDsSide, true));
    }
  }
  if (o.chuteUs) {
    this->log_ACTIVITY_HI_ChuteUsCommanded();
    if (this->isConnected_chuteCmdOut_OutputPort(0)) {
      this->chuteCmdOut_out(0, RocketTypes::ChuteCmd(this->nowSeconds(), ++m_chuteSeq, RocketTypes::Stage::US, false, true));
    }
  }
}

void FlightMode::schedIn_handler(FwIndexType, U32) {
  if (!m_warnedIncomplete) {
    this->log_WARNING_HI_LogicIncomplete();  // FM-05/13/15/17/18/20 填完後刪掉這行與事件
    m_warnedIncomplete = true;
  }
  m_in.t_s = this->nowSeconds();
  FmOutputs o;
  m_core.tick(m_in, m_cmd, o);
  m_cmd = FmGroundCmd();  // 地面指令是單拍
  this->emitOutputs(o);

  if (++m_tlmDiv >= kTlmDivider) {
    m_tlmDiv = 0;
    this->tlmWrite_mode(m_lastMode);
    this->tlmWrite_role(m_core.role() == FmRole::US ? FmRoleId::US : FmRoleId::DS);
  }
}

}  // namespace Mcu
