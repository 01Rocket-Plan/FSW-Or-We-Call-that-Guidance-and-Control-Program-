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
      m_lastMode(RocketTypes::FlightModeId::PAD_IDLE),
      m_sepSeq(0),
      m_tlmDiv(0),
      m_warnedShell(false) {}

FlightMode::~FlightMode() {}

F64 FlightMode::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void FlightMode::loadParams() {
  Fw::ParamValid v;
  const FmRoleId r = this->paramGet_role(v);
  const FmRole role = (r == FmRoleId::US) ? FmRole::US : FmRole::DS;
  if (role != m_core.role()) m_core.setRole(role);
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
  m_in.rate_rads = std::hypot(s.get_rate_body_rads()[0], s.get_rate_body_rads()[1]);
  m_in.imu_fresh = s.get_imuValid();
  m_in.velValid = s.get_velValid();
  m_in.alt_m = s.get_alt_m();
}

void FlightMode::uvIn_handler(FwIndexType, const RocketTypes::UvState& st) {
  m_in.uvState = static_cast<uint8_t>(st.e);
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
    // [FM-IN-7] 開傘驅動埠：repo 尚無（README「缺的介面」）
  }
  if (o.chuteUs) {
    this->log_ACTIVITY_HI_ChuteUsCommanded();
    // [FM-IN-7] 同上
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
