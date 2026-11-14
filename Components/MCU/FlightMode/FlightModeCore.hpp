// ======================================================================
// \title  FlightModeCore.hpp
// \author 殼：MCU 組（EdwardWang+Claude）｜邏輯：Edward 2026-11-14 授權 Claude 照 SSS 填
// \brief  FlightMode 純 C++ 核心（不含 F´、不含 ESP-IDF，主機可測）。
//
// 來源：SSS sss/fsw/flight_mode.py（2026-11-09 版），1:1 移植；與 SSS 不同的地方都標「repo 差異」。
// 填空狀態見 README.md「填空清單」。還沒填的格（等硬體／等定）：FM-05、13、15、17、20。
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

//! 同一份 tick、兩個部署（SSS ROLE_DS／ROLE_US）。repo 沒有 COMBINED：上下級各一顆 MCU、互不通訊。
enum class FmRole : uint8_t { DS = 0, US = 1 };

// ----------------------------------------------------------------------
// 發射架狀態（架 → 箭，臍帶 UART）。數值＝Components.PadState（PadLink.fpp）
// 與 PadLink::PadToRocketFlag（PadLinkCodec.hpp）。F´ 外殼用 static_assert 對過。
// ----------------------------------------------------------------------
namespace FmPad {
constexpr uint8_t SAFE = 0;
constexpr uint8_t ARMED = 1;
constexpr uint8_t COUNTDOWN = 2;
constexpr uint8_t IGNITION = 3;
constexpr uint8_t RELEASED = 4;
constexpr uint8_t LIFTED_OFF = 5;
constexpr uint8_t HANG_FIRE = 6;
constexpr uint8_t HOLD_ABORT = 7;
constexpr uint8_t NO_LIFTOFF = 8;

constexpr uint8_t FLAG_IGNITER_ON = 1u << 0;
constexpr uint8_t FLAG_THRUST_CONFIRMED = 1u << 1;
constexpr uint8_t FLAG_CLAMP_RELEASED = 1u << 2;
constexpr uint8_t FLAG_ROCKET_LINK_OK = 1u << 3;
}  // namespace FmPad

// ----------------------------------------------------------------------
// [FM-P] 參數：SSS FlightParams 的數字（2026-11-09 版）。HIL 可改，軟體必須有門檻。
// ----------------------------------------------------------------------
struct FmParams {
  float dt_s = 1.0e-3f;            //!< 一拍（1 kHz）
  float a_liftoff_mps2 = 5.0f;     //!< FM-08 離台加速度門檻
  float n_accel_s = 0.020f;        //!< FM-08 離台須持續
  float t_hang_s = 1.0f;           //!< FM-08 點火後沒離台 → HANGFIRE
  float t_fire_lead_s = 0.15f;     //!< FM-08 repo 差異：倒數剩這麼少就先閂 FIRE（臍帶 10 Hz＝0.1 s 一幀＋裕度）
  float v_active_mps = 8.0f;       //!< FM-10 被動 → 主動速度門檻
  float t_passive_min_s = 0.10f;   //!< FM-10 被動段最短
  float t_passive_max_s = 0.50f;   //!< FM-10 被動段最長（備援）
  float a_burnout_mps2 = -5.0f;    //!< FM-11 燃盡加速度門檻
  float n_burnout_s = 0.020f;      //!< FM-11 燃盡須持續
  float t_burnout_min_s = 0.40f;   //!< FM-11 離台後最短窗（防假燃盡）
  float t_burnout_max_s = 8.0f;    //!< FM-11 離台後最長窗
  float t_sep_inhibit_s = 0.80f;   //!< FM-12 進 COAST 後分離抑制窗
  float t_sep_backup_s = 4.0f;     //!< FM-12 進 COAST 後 4 s 分離（Edward 2026-11-14 暫定主路徑）
  float t_sep_fail_s = 5.0f;       //!< FM-13／17（未填）
  float t_chute_ds_s = 3.1f;       //!< FM-14 分離下令後下級開傘
  float t_chute_us_s = 3.2f;       //!< FM-18 上級開傘
  float t_land_max_s = 50.0f;      //!< FM-15（未填）
  float tilt_abort_rad = 0.5235988f;   //!< FM-06 30°
  float rate_abort_rads = 1.7453293f;  //!< FM-06 100°/s
  float t_abort_hold_s = 1.0f;     //!< FM-06 持續
  float t_abort_min_s = 0.20f;     //!< FM-06 離台後 tip-off 抑制
  float t_watchdog_s = 20.0f;      //!< FM-06 看門狗
  float t_imu_stale_s = 0.020f;    //!< FM-02 IMU 停更新門檻
  float a_high_mps2 = 5.0f;        //!< FM-14 accel_high＝accel_u > 這個（SSS 夾具定義）
};

// ----------------------------------------------------------------------
// 一拍的輸入。SSS Meas 欄位 → repo 來源，見 README「輸入對照」。
// ----------------------------------------------------------------------
struct FmInputs {
  double t_s = 0.0;  //!< F´ 時間服務

  // --- Estimator StateOut ---
  bool haveState = false;      //!< 收過 StateOut 沒
  float accel_u_mps2 = 0.0f;   //!< ← StateOut.accAxial_mps2
  float v_est_mps = 0.0f;      //!< ← |StateOut.vel_enu_mps|
  float vU_mps = 0.0f;         //!< ← StateOut.vel_enu_mps[2]
  float tilt_rad = 0.0f;       //!< ← StateOut.tilt_rad
  float rate_rads = 0.0f;      //!< ← hypot(rate_body_rads[1], rate_body_rads[2])
  bool imu_fresh = false;      //!< ← StateOut.imuValid
  bool velValid = false;       //!< ← StateOut.velValid
  float alt_m = 0.0f;          //!< ← StateOut.alt_m（只紀錄）

  // --- 電源 ---
  uint8_t uvState = 0;         //!< ← RocketTypes.UvState 數值

  // --- [FM-03] 發射架安檢（repo 差異：SSS 的 seated／loop_ok／ready／ground_link 四條件
  //     由發射架即時判定，箭只看架的結論）。← UmbilicalLink.padStatusOut ---
  bool padLinkOk = false;      //!< 臍帶架狀態夠新（UmbilicalLink padLinkTimeout_s）
  uint8_t padState = FmPad::SAFE;  //!< FmPad::*
  uint8_t padFlags = 0;        //!< FmPad::FLAG_*
  float padCountdown_s = 0.0f; //!< 架倒數剩餘秒（COUNTDOWN 時有效）

  // --- 離散訊號（驅動元件尚無；FM-05／13／15 等硬體）---
  bool sep_switch_open = false; //!< 分離面微動開關斷開（DiscreteSample.sepSwitchOpen）
  bool contact_ds = false;      //!< 下級著地
  bool contact_us = false;      //!< 上級著地
  bool sep_ok = false;          //!< FM-12 分離條件（門檻未定 gates D14；目前永遠 false → 走 4 s）
};

//! 地面指令（SSS GroundCmd）。repo：箭上地面指令走 LoRa＋UplinkAuth（HMAC）；
//! 外殼目前沒有來源（PIL 時接），全部 false。點火不在這裡：點火由架決定（gate C15）。
struct FmGroundCmd {
  bool abortValid = false;      //!< 已通過 UplinkAuth 的 ABORT（repo 差異：取代 SSS 密碼）
  bool override_ = false;
  bool gimbal_ds = false;
  bool gimbal_us = false;
  int requestMode = -1;         //!< <0 表示沒有
};

//! 跨拍記憶。欄位照 SSS ModeState。時間 <0 ＝ None。
struct FmState {
  FmMode ds = FmMode::PAD_IDLE;
  FmMode us = FmMode::US_WAIT;
  double t_fire_s = -1;
  double t_liftoff_s = -1;
  double t_coast_s = -1;
  double t_sep_cmd_s = -1;
  float t_hi_s = 0, t_lo_s = 0, t_tilt_s = 0, t_rate_s = 0, imu_stale_s = 0;
  bool motor_on = false;     //!< 只為對照 SSS；箭上沒有點火輸出（點火在架）
  bool sep_fired = false, chute_ds_fired = false, chute_us_fired = false;
  bool abort_done = false, fire_latched = false, us_chute_inhibit = false;
  bool pulse_sep = false, pulse_chute_ds = false, pulse_chute_us = false;
};

//! 事件（SSS FswEvent 的 name）。
enum class FmEvent : uint8_t {
  NONE = 0,
  MODE_CHANGE,
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

//! SSS 舵權（_authority）。repo 的 Controller 目前用模式自己判斷，這裡先算出來給遙測／對照。
enum class FmAuthority : uint8_t { CENTER = 0, CLOSED_LOOP = 1, LOCKED_CENTER = 2 };

//! [FM-19] 分離放哪幾顆：MG996R SEP_1–3（Servo kSepBits＝0x07）。
constexpr uint8_t kFmSepMaskAll = 0x07;

struct FmOutputs {
  FmMode mode = FmMode::PAD_IDLE;   //!< 這拍要廣播的模式（DS 部署＝ds；US 部署＝us）
  FmAuthority authority = FmAuthority::CENTER;
  bool sepFire = false;             //!< 單拍脈衝 → SepCmd
  uint8_t sepMask = 0;
  bool chuteDs = false;             //!< 單拍脈衝 → chuteCmdOut（驅動元件尚無）
  bool chuteDsSide = false;         //!< SEP_FAIL 側傘
  bool chuteUs = false;
  FmEvent events[kFmMaxEvents] = {};
  int nEvents = 0;
};

class FlightModeCore {
 public:
  void setParams(const FmParams& p) { m_p = p; }
  const FmParams& params() const { return m_p; }
  void setRole(FmRole r);
  FmRole role() const { return m_role; }
  const FmState& state() const { return m_s; }
  //! 測試用：直接設定跨拍狀態（SSS 測試的 ModeState(ds=COAST, ...) 寫法）
  void setStateForTest(const FmState& s) { m_s = s; }

  //! 每拍一次（1 kHz）。
  void tick(const FmInputs& in, const FmGroundCmd& cmd, FmOutputs& out);

 private:
  void tickUs(const FmInputs& in, const FmGroundCmd& cmd, FmOutputs& out);
  void setDs(FmMode m, FmOutputs& out);
  void setUs(FmMode m, FmOutputs& out);
  void enterLiftoff(double t, FmOutputs& out);
  void maybeCoast(const FmInputs& in, double t, FmOutputs& out);
  void maybeSep(const FmInputs& in, double t, FmOutputs& out);
  void maybeChuteDs(const FmInputs& in, double t, FmOutputs& out);
  void enterAbort(double t, FmOutputs& out);
  void advanceUs(const FmInputs& in, double t, FmOutputs& out);
  FmAuthority authority() const;
  bool accelHigh(const FmInputs& in) const { return in.accel_u_mps2 > m_p.a_high_mps2; }
  static void addEvent(FmOutputs& out, FmEvent e);

  FmParams m_p;
  FmRole m_role = FmRole::DS;
  FmState m_s;
};

}  // namespace Mcu

#endif
