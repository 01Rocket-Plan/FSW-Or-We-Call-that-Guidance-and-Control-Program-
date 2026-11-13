// ======================================================================
// \title  FlightModeCore.cpp
// \author 殼：MCU 組（EdwardWang+Claude）｜邏輯：Edward 本人
// \brief  tick() 的骨架。順序照 SSS flight_mode.py tick() 的「一拍順序」：
//         清脈衝 → 累 IMU 過期 → 濾地面指令 → 假分禁傘 → D12 中止
//         → 依 ds 走一條邊 → 若負責上級則推進 us → 組輸出
//
// 每個 [FM-xx] 都是空的。要填時在 issue／PR 說「填 FM-xx」即可，
// README.md「填空清單」寫了每格對應 SSS 的哪個函式、哪幾行。
// ======================================================================
#include "Components/MCU/FlightMode/FlightModeCore.hpp"

namespace Mcu {

void FlightModeCore::setRole(FmRole r) {
  m_role = r;
  m_s = FmState();  // 換部署 → 回初始（DS: PAD_IDLE，US: US_WAIT）
}

void FlightModeCore::tick(const FmInputs& in, const FmGroundCmd& cmd, FmOutputs& out) {
  (void)in;
  (void)cmd;
  out = FmOutputs();

  if (m_role == FmRole::US) {
    // [FM-18] 上級部署：SSS _tick_us()（L478–518）
    //         級間鏈路收 sep_cmd／inhibit → 假分判斷 → US_WAIT／US_COAST／US_CHUTE 推進
    out.mode = m_s.us;
    return;
  }

  // [FM-01] 清脈衝：pulse_sep／pulse_chute_ds／pulse_chute_us = false（SSS L192–194）
  // [FM-02] IMU 過期累計：imu_fresh=false → imu_stale_s += dt，否則歸零（SSS L199–202）
  // [FM-03] 解保險互鎖：seated ∧ loop_ok ∧ ready ∧ ground_link（SSS L204）
  // [FM-04] 地面指令過濾：override／gimbal／中止密碼／request_mode LEGAL 表（SSS L207–223）
  // [FM-05] 假分禁傘：sep_switch_open ∧ ¬sep_fired → us_chute_inhibit（SSS L225–227）
  // [FM-06] D12 自動中止：傾角／角速度持續 t_abort_hold_s、看門狗 t_watchdog_s（SSS L229–252）

  switch (m_s.ds) {
    case FmMode::PAD_IDLE:       // [FM-07] PAD_IDLE → ARMED（SSS L255–259）
      break;
    case FmMode::ARMED:          // [FM-08] 互鎖掉線回 PAD_IDLE；FIRE 閂鎖；離台偵測 → LIFTOFF；逾時 → HANGFIRE（SSS L260–281）
      break;
    case FmMode::HANGFIRE:       // [FM-08b] 地面安全態，丟 FIRE（SSS L282–285）
      break;
    case FmMode::LIFTOFF:        // [FM-09] 下一拍 → BOOST_PASSIVE（SSS L286–288）
      break;
    case FmMode::BOOST_PASSIVE:  // [FM-10] v_est > v_active ∧ ≥ t_passive_min，或 ≥ t_passive_max → BOOST_ACTIVE（SSS L289–293）
      break;
    case FmMode::BOOST_ACTIVE:   // [FM-11] 燃盡 → COAST：SSS _maybe_coast()（L374–391）
      break;
    case FmMode::COAST:          // [FM-12] 分離：SSS _maybe_sep()（L393–410）；sep_ok 門檻未定（gates D14）
      break;
    case FmMode::SEPARATED:      // [FM-13] 開關斷 → LS_DESCENT（並推上級）；逾時 → SEP_FAIL（SSS L298–307）
      break;
    case FmMode::LS_DESCENT:     // [FM-14] 下級開傘：SSS _maybe_chute_ds()（L412–427）
      break;
    case FmMode::LS_CHUTE:       // [FM-15] 著地或 t_land_max → LS_LANDED（SSS L310–315）
      break;
    case FmMode::SEP_FAIL:       // [FM-16] 側傘（SSS L316–317）
      break;
    case FmMode::ABORT:          // [FM-17] 分離失敗 → SEP_FAIL；分得開 → LS_DESCENT（SSS L318–326）
      break;
    default:
      break;
  }

  // [FM-18] COMBINED 時推進上級：SSS _advance_us()（L455–476）。DS 部署不推（上級在另一顆 MCU）。

  // [FM-19] 組輸出：mode、sepFire／sepMask、chuteDs、bus_*（SSS L331–350）
  out.mode = m_s.ds;
}

}  // namespace Mcu
