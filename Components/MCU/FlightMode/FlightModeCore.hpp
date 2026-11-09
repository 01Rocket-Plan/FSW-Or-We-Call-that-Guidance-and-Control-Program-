// ======================================================================
// \title  FlightModeCore.hpp
// \author 殼：MCU 組（EdwardWang+Claude）｜邏輯：Edward 本人
// \brief  FlightMode 純 C++ 核心「殼」。
//
// ⚠️ CONTRIBUTING 第 7 條：轉移邏輯、觸發、抑制窗、計時器由 Edward 撰寫。
//    本檔只有資料結構與空的 tick()；每個要填的地方標 [FM-xx]，
//    對照表在 README.md「填空清單」，來源是 SSS sss/fsw/flight_mode.py（2026-11-09 版）。
//
// 殼的預設行為（安全）：DS 永遠 PAD_IDLE、US 永遠 US_WAIT；不分離、不開傘、不寫 NV。
// ======================================================================
#ifndef COMPONENTS_MCU_FLIGHTMODE_FLIGHTMODECORE_HPP
#define COMPONENTS_MCU_FLIGHTMODE_FLIGHTMODECORE_HPP

#include <cstdint>

namespace Mcu {

//! 數值＝RocketTypes.FlightModeId（PR #26）。改一邊就要改另一邊。
enum class FmMode : uint8_t {
  PAD_IDLE = 0,
  ARMED = 1,
  LIFTOFF = 2,
  BOOST_PASSIVE = 3,
  BOOST_ACTIVE = 4,
  COAST = 5,
  SEPARATED = 6,
  LS_DESCENT = 7,
  LS_CHUTE = 8,
  LS_LANDED = 9,
  HANGFIRE = 10,
  SEP_FAIL = 11,
  ABORT = 12,
  US_WAIT = 13,
  US_COAST = 14,
  US_CHUTE = 15,
  US_LANDED = 16,
  US_CHUTE_INHIBIT = 17,
};

//! 同一份 tick、兩個部署（SSS ROLE_DS／ROLE_US）。
enum class FmRole : uint8_t { DS = 0, US = 1 };

// ----------------------------------------------------------------------
// [FM-P] 參數：照 SSS FlightParams 填（數字表在 README「FM-P」）。殼裡刻意留空。
// ----------------------------------------------------------------------
struct FmParams {
  float dt_s = 1.0e-3f;  //!< 一拍（1 kHz）；只有這個不是觸發門檻
  // [FM-P] a_liftoff_mps2, n_accel_s, t_hang_s, v_active_mps, t_passive_min_s, t_passive_max_s,
  //        a_burnout_mps2, n_burnout_s, t_burnout_min_s, t_burnout_max_s, t_sep_inhibit_s,
  //        t_sep_backup_s, t_sep_fail_s, t_chute_ds_s, t_chute_us_s, t_land_max_s,
  //        tilt_abort_rad, rate_abort_rads, t_abort_hold_s, t_abort_min_s, t_watchdog_s,
  //        t_imu_stale_s, (sep_ok 門檻：未定，gates D14)
};

// ----------------------------------------------------------------------
// 一拍的輸入。SSS Meas 欄位 → repo 來源，見 README「輸入對照」。
// 「有來源」的欄位外殼已經填好；「缺」的欄位外殼填安全值（見註解）。
// ----------------------------------------------------------------------
struct FmInputs {
  double t_s = 0.0;  //!< F´ 時間服務

  // --- 有來源：Estimator StateOut ---
  bool haveState = false;      //!< 收過 StateOut 沒
  float accel_u_mps2 = 0.0f;   //!< ← StateOut.accAxial_mps2（世界 U 向運動學加速度）
  float v_est_mps = 0.0f;      //!< ← |StateOut.vel_enu_mps|
  float vU_mps = 0.0f;         //!< ← StateOut.vel_enu_mps[2]
  float tilt_rad = 0.0f;       //!< ← StateOut.tilt_rad
  float rate_rads = 0.0f;      //!< ← hypot(rate_body_rads[0], rate_body_rads[1])（橫向，不含滾轉）
  bool imu_fresh = false;      //!< ← StateOut.imuValid
  bool velValid = false;       //!< ← StateOut.velValid
  float alt_m = 0.0f;          //!< ← StateOut.alt_m（只紀錄；SSS：氣壓不觸發轉移）

  // --- 有來源：電源 ---
  uint8_t uvState = 0;         //!< ← RocketTypes.UvState 數值

  // --- 缺（README「缺的介面」）：外殼一律填「不放行」的安全值 ---
  bool seated = false;          //!< [FM-IN-1] 箭在架上
  bool loop_ok = false;         //!< [FM-IN-1] 點火迴路正常
  bool ready = false;           //!< [FM-IN-1] 箭體 READY
  bool ground_link = false;     //!< [FM-IN-1] 地面鏈路
  bool sep_switch_open = false; //!< [FM-IN-2] 分離面微動開關斷開
  bool contact_ds = false;      //!< [FM-IN-3] 下級著地
  bool contact_us = false;      //!< [FM-IN-3] 上級著地
  bool sep_ok = false;          //!< [FM-IN-4] 分離條件（門檻未定，gates D14）
  bool link_alive = false;      //!< [FM-IN-5] 級間鏈路（US 部署用）
  bool link_sep_cmd = false;    //!< [FM-IN-5]
  double link_t_sep_cmd_s = -1; //!< [FM-IN-5] <0 表示沒有
  bool link_us_inhibit = false; //!< [FM-IN-5]
};

//! 地面指令（SSS GroundCmd）。來源 [FM-IN-6]：目前 repo 沒有送到 FlightMode 的路徑。
struct FmGroundCmd {
  bool arm = false;
  bool fire = false;            //!< ⚠️ repo gate C15：點火在發射架，不在箭上（README「衝突」）
  bool abortValid = false;      //!< ⚠️ SSS 用密碼；repo 走 UplinkAuth HMAC（README「衝突」）
  bool override_ = false;
  bool gimbal_ds = false;
  bool gimbal_us = false;
  int requestMode = -1;         //!< <0 表示沒有
};

//! 跨拍記憶。欄位照 SSS ModeState 一一對應（只有資料，沒有邏輯）。
struct FmState {
  FmMode ds = FmMode::PAD_IDLE;
  FmMode us = FmMode::US_WAIT;
  double t_fire_s = -1;      //!< <0 = None
  double t_liftoff_s = -1;
  double t_coast_s = -1;
  double t_sep_cmd_s = -1;
  float t_hi_s = 0, t_lo_s = 0, t_tilt_s = 0, t_rate_s = 0, imu_stale_s = 0;
  bool motor_on = false;
  bool sep_fired = false, chute_ds_fired = false, chute_us_fired = false;
  bool abort_done = false, fire_latched = false, us_chute_inhibit = false;
  bool pulse_sep = false, pulse_chute_ds = false, pulse_chute_us = false;
};

//! 事件（SSS FswEvent 的 name）。外殼轉成 F´ 事件。
enum class FmEvent : uint8_t {
  NONE = 0,
  MODE_CHANGE,      //!< SSS "MODE_*"
  REJECT_OVERRIDE,
  DROP_GIMBAL_DS,
  GIMBAL_US,
  REJECT_ABORT,
  REJECT_TRANS,
  DROP_FIRE,
  CMD_SEP,
  CMD_CHUTE_DS,
  CMD_CHUTE_US,
  SIDE_CHUTE,
  SEP_FAIL,
};

constexpr int kFmMaxEvents = 8;

struct FmOutputs {
  FmMode mode = FmMode::PAD_IDLE;   //!< 這拍要廣播的模式（DS 部署＝ds；US 部署＝us）
  bool sepFire = false;             //!< 單拍脈衝 → SepCmd
  uint8_t sepMask = 0;              //!< [FM-19] 要放哪幾顆 MG996R（SSS 沒有，硬體 3 顆）
  bool chuteDs = false;             //!< 單拍脈衝 → [FM-IN-7] 開傘驅動（缺）
  bool chuteUs = false;
  bool busSepCmd = false;           //!< 級間匯流排（SSS bus_*）
  double busTSepCmd_s = -1;
  bool busUsInhibit = false;
  FmEvent events[kFmMaxEvents] = {};
  int nEvents = 0;
};

class FlightModeCore {
 public:
  void setParams(const FmParams& p) { m_p = p; }
  void setRole(FmRole r);
  FmRole role() const { return m_role; }
  const FmState& state() const { return m_s; }

  //! 每拍一次（1 kHz）。殼：只輸出目前模式，什麼都不做。
  void tick(const FmInputs& in, const FmGroundCmd& cmd, FmOutputs& out);

 private:
  FmParams m_p;
  FmRole m_role = FmRole::DS;
  FmState m_s;
};

}  // namespace Mcu

#endif
