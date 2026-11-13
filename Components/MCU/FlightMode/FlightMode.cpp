// ======================================================================
// \title  FlightMode.cpp
// \author 殼：MCU 組（EdwardWang+Claude）｜邏輯：Edward 本人
// \brief  F´ 外殼：搬輸入 → core.tick() → 廣播模式、送 SepCmd、寫 NV 閂鎖、事件。
//         這裡只有「搬資料」與「把 core 的輸出送出去」，沒有任何轉移判斷。
// ======================================================================
#include "Components/MCU/FlightMode/FlightMode.hpp"

#include <cmath>

namespace Mcu {

namespace {
constexpr U32 kTlmDivider = 100;  // 1 kHz → 10 Hz

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
      m_linkSeq(0),
      m_tlmDiv(0),
      m_warnedShell(false) {}

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

// [FM-IN-5] 只搬資料。對端逾時由 Edward 在 tick 裡判斷（看 t_s）。
void FlightMode::stageLinkIn_handler(FwIndexType, const RocketTypes::StageLinkMsg& m) {
  m_in.link_alive = m.get_alive();
  m_in.link_sep_cmd = m.get_sepCmd();
  m_in.link_t_sep_cmd_s = m.get_tSepCmd_s();
  m_in.link_us_inhibit = m.get_usInhibit();
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
      // side（SEP_FAIL 側傘）由 Edward 決定何時為 true；殼一律 false
      this->chuteCmdOut_out(0, RocketTypes::ChuteCmd(this->nowSeconds(), ++m_chuteSeq, RocketTypes::Stage::DS, false, true));
    }
  }
  if (o.chuteUs) {
    this->log_ACTIVITY_HI_ChuteUsCommanded();
    if (this->isConnected_chuteCmdOut_OutputPort(0)) {
      this->chuteCmdOut_out(0, RocketTypes::ChuteCmd(this->nowSeconds(), ++m_chuteSeq, RocketTypes::Stage::US, false, true));
    }
  }
  // [FM-IN-5] 級間匯流排：DS 部署每拍送（alive＝本端活著）
  if (m_core.role() == FmRole::DS && this->isConnected_stageLinkOut_OutputPort(0)) {
    this->stageLinkOut_out(0, RocketTypes::StageLinkMsg(this->nowSeconds(), ++m_linkSeq, true, o.busSepCmd,
                                                         o.busTSepCmd_s, o.busUsInhibit));
  }
}

void FlightMode::schedIn_handler(FwIndexType, U32) {
  if (!m_warnedShell) {
    this->log_WARNING_HI_ShellOnly();  // 殼提醒：填完 FM-01..FM-19 後刪掉這行與事件
    m_warnedShell = true;
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
