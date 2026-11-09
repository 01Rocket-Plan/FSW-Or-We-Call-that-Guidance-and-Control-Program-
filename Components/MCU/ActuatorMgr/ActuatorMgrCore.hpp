// ======================================================================
// \title  ActuatorMgrCore.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  DS 作動管理 W4。純 C++14，不含 F´、不 sleep、不讀系統時間。
//
// 尾翼路（每拍）：
//   fin_rad[i] → 速率限制（rateMax_rads）→ 限幅（±finMax_rad）
//             → 舵機角 = dir[i]·gear·θ + trim[i] → µs = neutral + 舵機角·usPerRad
//             → 硬限 [pulseMin, pulseMax]（conventions：1000–2000 µs）
//   鎖中位（任一成立）：FinCmd.lockNeutral、FinCmd 過期、欠壓 ≥ uvLockLevel、從沒收過 FinCmd
//     → 四路 = neutral + trim·usPerRad，速率限制狀態歸零（解鎖後從 0 平滑起步）
//
// 分離路：
//   FlightMode 的 SepCmd（valid、seq 新）→ 轉給 SepServoDriver。
//   sepStagger_s > 0 時依位元順序一顆一顆放（SSS §10「三顆 MG996R 錯開 Δ ms」）。
//   欠壓**不**擋分離（gates：欠壓 → 舵中立、分離不鎖）。fire-once 由 SepServoDriver＋NvFlags 管。
// ======================================================================
#ifndef COMPONENTS_MCU_ACTUATORMGR_ACTUATORMGRCORE_HPP
#define COMPONENTS_MCU_ACTUATORMGR_ACTUATORMGRCORE_HPP

#include <cstdint>

namespace Mcu {

constexpr int kFinCount = 4;

struct ActuatorMgrParams {
  uint16_t neutral_us = 1500;       //!< SG90_NEUTRAL_US
  uint16_t pulseMin_us = 1000;      //!< SG90_PULSE_MIN_US
  uint16_t pulseMax_us = 2000;      //!< SG90_PULSE_MAX_US
  float usPerRad = 318.30989f;      //!< SG90：1000 µs / 180°（Sg90Core kTravelDeg 假設，未實測）
  float gear = 1.0f;                //!< 舵機角 / 尾翼角（連桿比）
  float dir[kFinCount] = {1, 1, 1, 1};     //!< 每通道正反向（±1），冷測定
  float trim_rad[kFinCount] = {0, 0, 0, 0};  //!< 每通道機械中位配平（舵機角）
  float finMax_rad = 0.5235988f;    //!< ±30°（與 Controller 相同，這裡再擋一次）
  float rateMax_rads = 10.47f;      //!< SG90 約 0.1 s/60°（4.8 V 規格書）
  float finStale_s = 0.020f;        //!< FinCmd 多舊就鎖中位
  uint8_t uvLockLevel = 2;          //!< RocketTypes.UvState 數值；≥ 此值鎖中位（2 = UV_SHED）
  float sepStagger_s = 0.0f;        //!< 分離通道錯開時間；0 = 同時
  float dtMax_s = 0.005f;
};

enum class LockReason : uint8_t {
  NONE = 0,
  NO_CMD = 1,      //!< 從沒收過 FinCmd
  CMD_LOCK = 2,    //!< FinCmd.lockNeutral
  CMD_STALE = 3,   //!< FinCmd 過期
  UNDERVOLT = 4,   //!< 欠壓
};

struct FinIn {
  double t_s = 0.0;
  uint32_t seq = 0;
  float fin_rad[kFinCount] = {0, 0, 0, 0};
  bool lockNeutral = true;
};

struct SepIn {
  double t_s = 0.0;
  uint32_t seq = 0;
  uint8_t channelMask = 0;
  bool valid = false;
};

struct ServoOut {
  double t_s = 0.0;
  uint32_t seq = 0;
  uint16_t pulse_us[kFinCount] = {1500, 1500, 1500, 1500};
  bool lockNeutral = true;  //!< 只在「進入鎖定」那一拍為 true（見 README：避免驅動事件洪水）
  bool locked = true;       //!< 這拍是否處於鎖中位（遙測）
  LockReason reason = LockReason::NO_CMD;
  uint32_t clampCount = 0;  //!< 累計 µs 硬限次數
  uint32_t rateLimitCount = 0;
};

struct SepOut {
  bool send = false;  //!< 這拍要不要送
  double t_s = 0.0;
  uint32_t seq = 0;
  uint8_t channelMask = 0;
};

class ActuatorMgrCore {
 public:
  void setParams(const ActuatorMgrParams& p) { m_p = p; }
  const ActuatorMgrParams& params() const { return m_p; }

  void pushFin(const FinIn& c) {
    m_fin = c;
    m_haveFin = true;
  }
  void setUv(uint8_t uvState) { m_uv = uvState; }
  //! 收 SepCmd。valid=false 或 seq 不是新的 → 丟棄並回 false。
  bool pushSep(const SepIn& c, double now_s);

  //! 每拍一次。
  void step(double now_s, ServoOut& servo, SepOut& sep);

  uint8_t pendingSepMask() const { return m_sepPending; }
  uint32_t sepDropCount() const { return m_sepDrop; }

 private:
  uint16_t toPulse(int ch, float servoAngle_rad, bool& clamped) const;

  ActuatorMgrParams m_p;
  FinIn m_fin;
  bool m_haveFin = false;
  uint8_t m_uv = 0;
  float m_theta[kFinCount] = {0, 0, 0, 0};  //!< 速率限制後的尾翼角
  bool m_wasLocked = false;
  bool m_firstStep = true;
  double m_lastT_s = 0.0;
  uint32_t m_seq = 0;
  uint32_t m_clamp = 0;
  uint32_t m_rateLim = 0;

  bool m_haveSepSeq = false;
  uint32_t m_lastSepSeq = 0;
  uint8_t m_sepPending = 0;
  double m_sepNext_s = 0.0;
  uint32_t m_sepOutSeq = 0;
  uint32_t m_sepDrop = 0;
};

}  // namespace Mcu

#endif
