// ======================================================================
// \title  ControllerCore.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  DS 姿控 W3：StateOut＋舵權 → 四片尾翼角（rad）。
//         純 C++14，不含 F´、不 sleep、不讀系統時間。
//
// 控制律：PD，增益與限幅沿用 SSS FlightParams（kp_fin 1.2、kd_fin 0.25、±30°）。
// 座標：conventions §2（2026-11-10 業界標準）：機體 X 指鼻錐（滾轉）、Y 右（俯仰）、Z = X×Y（偏航）。
//
//   r   = 發射參考：世界座標下的目標鼻向 = R(q_ref)·x̂   （Status #8；沒捕捉前 = 世界 +U (0,0,1)）
//   u_b = r 在機體的分量 = R(q)ᵀ·r                     （姿態 = 參考時 = (1,0,0)）
//   e   = x̂ × u_b = (0, −u_z, u_y)                    （把鼻轉回參考需要的轉軸；e_y≈−pitch、e_z≈+yaw，相對參考）
//   δy  = sat(kp·e_y − kd·ω_y)（俯仰），δz = sat(kp·e_z − kd·ω_z)（偏航）
//   fin = (δy, δz, −δy, −δz)                           （= SSS rigid_body.py：1/3 管 My、2/4 管 Mz）
//
// 「+δy」的意思＝要產生 +Y 力矩的舵量。實體舵機正反向由 ActuatorMgr 每通道符號處理，
// 冷測（SSS §5 冷測 2；selftest ST-ATT-PITCH-SIGN／YAW-SIGN）驗證，不改這裡的符號。
// 不控滾轉（ω_x 不進控制律；SSS 也沒有）。
//
// 發射參考（Status #8，方案 a）：第一次進 BOOST_ACTIVE、且舵權前四關（有模式、有狀態、狀態新鮮、
// imuValid）都過的第一拍，把當下 q 凍結成 q_ref，存 r = R(q_ref)·x̂。之後 BOOST_ACTIVE／COAST
// 都對 r 閉環。imuValid 在進 BOOST 那拍是 false → 在 BOOST 內第一個合格拍才捕捉（那之前本來就不閉環）。
// 回到 PAD_IDLE／ARMED（CtlMode::PRE_LAUNCH）就清掉參考 → 重新 ARMED 後的下一次 BOOST 重新捕捉；
// 重開機 core 重建，參考自然清掉。沒捕捉到參考時退回世界垂直（= 舊行為）。
// 直立發射時 r = (0,0,1)，控制律跟改之前完全一樣。
// ======================================================================
#ifndef COMPONENTS_MCU_CONTROLLER_CONTROLLERCORE_HPP
#define COMPONENTS_MCU_CONTROLLER_CONTROLLERCORE_HPP

#include <cstdint>

namespace Mcu {

struct ControllerParams {
  float kp = 1.2f;              //!< rad 舵 / rad 誤差（SSS kp_fin）
  float kd = 0.25f;             //!< rad 舵 / (rad/s)（SSS kd_fin）
  float finMax_rad = 0.5235988f;  //!< ±30°（SSS fin_max_rad）
  float stateStale_s = 0.020f;  //!< StateOut 多舊就不閉環（對齊 SSS t_imu_stale_s）
  bool demo50mFixed = false;    //!< A9 Demo-1-1：主動段舵恆 0（SSS demo_50m_fixed）
};

//! 為什麼沒閉環（遙測用；CLOSED_LOOP 時為 NONE）。
enum class HoldReason : uint8_t {
  NONE = 0,          //!< 閉環中
  NO_MODE = 1,       //!< 還沒收過 ModePort
  MODE_INACTIVE = 2, //!< 模式不是 BOOST_ACTIVE／COAST
  NO_STATE = 3,      //!< 還沒收過 StateOut
  STATE_STALE = 4,   //!< StateOut 太舊
  IMU_INVALID = 5,   //!< StateOut.imuValid == false（IMU 過期或未對準）
  DEMO_FIXED = 6,    //!< demo50mFixed 開著
};

//! Controller 只需要知道的模式分類。由外殼依 FlightModeId 對映（Controller::toCtlMode）；
//! core 只讀模式，不做任何模式轉換（CONTRIBUTING 第 7 條）。
enum class CtlMode : uint8_t {
  PRE_LAUNCH = 0,    //!< PAD_IDLE／ARMED：不閉環，清掉發射參考
  BOOST_ACTIVE = 1,  //!< 閉環；第一個合格拍捕捉發射參考
  COAST = 2,         //!< 閉環；沿用已捕捉的參考
  INACTIVE = 3,      //!< 其他模式（LIFTOFF、BOOST_PASSIVE、ABORT…）：不閉環，參考保持不動
};

struct StateIn {
  double t_s = 0.0;
  uint32_t seq = 0;
  float q_wxyz[4] = {1, 0, 0, 0};
  float rate_body_rads[3] = {0, 0, 0};
  bool imuValid = false;
};

struct FinOut {
  double t_s = 0.0;
  uint32_t seq = 0;
  float fin_rad[4] = {0, 0, 0, 0};
  bool lockNeutral = true;
  HoldReason reason = HoldReason::NO_MODE;
  float err_rad[2] = {0, 0};  //!< e_y（俯仰）, e_z（偏航）（遙測）
};

class ControllerCore {
 public:
  void setParams(const ControllerParams& p) { m_p = p; }
  const ControllerParams& params() const { return m_p; }

  void pushState(const StateIn& s) {
    m_state = s;
    m_haveState = true;
  }
  //! 目前模式（外殼每拍由 FlightModeId 對映後送進來）。BOOST_ACTIVE／COAST 才可閉環；
  //! PRE_LAUNCH 會清掉發射參考。
  void setMode(CtlMode mode) {
    m_mode = mode;
    m_activeMode = (mode == CtlMode::BOOST_ACTIVE || mode == CtlMode::COAST);
    m_haveMode = true;
    if (mode == CtlMode::PRE_LAUNCH) clearLaunchRef();
  }

  //! 發射參考是否已捕捉；r = 世界座標的目標鼻向（單位向量；沒捕捉時 (0,0,1)）。
  bool launchRefCaptured() const { return m_refCaptured; }
  const float* launchRef() const { return m_ref; }

  //! 每拍一次。
  void step(double now_s, FinOut& out);

  //! 純 PD（測試用，公開）：給姿態與角速度，對目前的發射參考 r 回 δy、δz（已限幅）與誤差。
  void pd(const float q[4], const float w[3], float& dy, float& dz, float err[2]) const;

 private:
  void captureLaunchRef(const float q[4]);
  void clearLaunchRef() {
    m_refCaptured = false;
    m_ref[0] = 0.0f;
    m_ref[1] = 0.0f;
    m_ref[2] = 1.0f;
  }

  ControllerParams m_p;
  StateIn m_state;
  bool m_haveState = false;
  bool m_haveMode = false;
  bool m_activeMode = false;
  CtlMode m_mode = CtlMode::PRE_LAUNCH;
  bool m_refCaptured = false;
  float m_ref[3] = {0.0f, 0.0f, 1.0f};  //!< 世界 ENU；預設 +U（垂直）
  uint32_t m_seq = 0;
};

}  // namespace Mcu

#endif
