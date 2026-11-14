// ======================================================================
// \title  FlightModeCore.cpp
// \author 殼：MCU 組（EdwardWang+Claude）｜邏輯：Edward 2026-11-14 授權 Claude 照 SSS 填
// \brief  tick()。順序照 SSS flight_mode.py tick() 的「一拍順序」：
//         清脈衝 → 累 IMU 過期 → 濾地面指令 → 假分禁傘 → D12 中止
//         → 依 ds 走一條邊 → 組輸出
//
// 1:1 移植 SSS（2026-11-09 版，行號以那版為準）。與 SSS 不同處標「repo 差異」。
// 分離開關、著地接觸在這裡只是輸入；SIL／SSS 用模擬值餵，硬體照這份邏輯做。
// 還沒填：FM-20（重啟規則待定）。
// ======================================================================
#include "Components/MCU/FlightMode/FlightModeCore.hpp"

namespace Mcu {

namespace {

//! SSS _FLIGHT：離台後（含中止／分離失敗）。HANGFIRE 不在這裡。
bool inFlightSet(FmMode m) {
  switch (m) {
    case FmMode::LIFTOFF:
    case FmMode::BOOST_PASSIVE:
    case FmMode::BOOST_ACTIVE:
    case FmMode::COAST:
    case FmMode::SEPARATED:
    case FmMode::LS_DESCENT:
    case FmMode::LS_CHUTE:
    case FmMode::LS_LANDED:
    case FmMode::ABORT:
    case FmMode::SEP_FAIL:
      return true;
    default:
      return false;
  }
}

//! SSS _ABORTABLE：D12 只在這四態生效。
bool abortableSet(FmMode m) {
  return m == FmMode::LIFTOFF || m == FmMode::BOOST_PASSIVE || m == FmMode::BOOST_ACTIVE ||
         m == FmMode::COAST;
}

//! SSS LEGAL：地面 request_mode 允許的邊。只用來決定記不記 REJECT_TRANS，合法也不切模式。
bool legalEdge(FmMode from, int to) {
  struct Edge { FmMode a; FmMode b; };
  static const Edge kLegal[] = {
      {FmMode::PAD_IDLE, FmMode::ARMED},           {FmMode::ARMED, FmMode::PAD_IDLE},
      {FmMode::ARMED, FmMode::LIFTOFF},            {FmMode::ARMED, FmMode::HANGFIRE},
      {FmMode::LIFTOFF, FmMode::BOOST_PASSIVE},    {FmMode::LIFTOFF, FmMode::ABORT},
      {FmMode::BOOST_PASSIVE, FmMode::BOOST_ACTIVE}, {FmMode::BOOST_PASSIVE, FmMode::ABORT},
      {FmMode::BOOST_ACTIVE, FmMode::COAST},       {FmMode::BOOST_ACTIVE, FmMode::ABORT},
      {FmMode::COAST, FmMode::SEPARATED},          {FmMode::COAST, FmMode::ABORT},
      {FmMode::SEPARATED, FmMode::LS_DESCENT},     {FmMode::SEPARATED, FmMode::SEP_FAIL},
      {FmMode::LS_DESCENT, FmMode::LS_CHUTE},      {FmMode::LS_CHUTE, FmMode::LS_LANDED},
      {FmMode::ABORT, FmMode::SEP_FAIL},           {FmMode::ABORT, FmMode::LS_DESCENT},
  };
  for (const Edge& e : kLegal) {
    if (e.a == from && static_cast<int>(e.b) == to) return true;
  }
  return false;
}

//! 架已接受 ARM、仍在走發射流程（含點火後到離台）。
bool padInLaunchFlow(uint8_t st) {
  return st == FmPad::ARMED || st == FmPad::COUNTDOWN || st == FmPad::IGNITION ||
         st == FmPad::RELEASED || st == FmPad::LIFTED_OFF;
}

//! 架回報點火後失敗（沒推力／夾住中止／沒離台）。
bool padFailed(uint8_t st) {
  return st == FmPad::HANG_FIRE || st == FmPad::HOLD_ABORT || st == FmPad::NO_LIFTOFF;
}

}  // namespace

void FlightModeCore::setRole(FmRole r) {
  m_role = r;
  m_s = FmState();  // 換部署 → 回初始（DS: PAD_IDLE，US: US_WAIT）
}

void FlightModeCore::addEvent(FmOutputs& out, FmEvent e) {
  if (out.nEvents < kFmMaxEvents) out.events[out.nEvents++] = e;
}

void FlightModeCore::setDs(FmMode m, FmOutputs& out) {
  if (m_s.ds == m) return;  // 模式沒變就不重複記
  m_s.ds = m;
  addEvent(out, FmEvent::MODE_CHANGE);
}

void FlightModeCore::setUs(FmMode m, FmOutputs& out) {
  if (m_s.us == m) return;
  m_s.us = m;
  addEvent(out, FmEvent::MODE_CHANGE);
}

void FlightModeCore::tick(const FmInputs& in, const FmGroundCmd& cmd, FmOutputs& out) {
  out = FmOutputs();
  const double t = in.t_s;

  if (m_role == FmRole::US) {
    tickUs(in, cmd, out);
    return;
  }

  // [FM-01] 清脈衝（SSS L192–194）
  m_s.pulse_sep = false;
  m_s.pulse_chute_ds = false;
  m_s.pulse_chute_us = false;

  // [FM-02] IMU 過期累計（SSS L199–202）
  if (!in.imu_fresh) {
    m_s.imu_stale_s += m_p.dt_s;
  } else {
    m_s.imu_stale_s = 0.0f;
  }

  // [FM-03] 解保險互鎖（SSS L204）。
  // repo 差異：SSS 的四條件（seated、點火迴路、READY、地面鏈路）由發射架即時安檢
  // （PadController kPadPermitMask：到位、迴路、鑰匙、架傾角、導軌、架電、地面鏈路、臍帶、箭 READY 全綠），
  // 架只有全綠才會進 ARMED／COUNTDOWN，任一紅燈就退回 SAFE。箭只看架的結論：
  //   臍帶架狀態夠新 ∧ 架有收到箭（ROCKET_LINK_OK）∧ 架還在發射流程裡。
  const bool padFresh = in.padLinkOk && (in.padFlags & FmPad::FLAG_ROCKET_LINK_OK) != 0;
  const bool interlock = padFresh && padInLaunchFlow(in.padState);
  // SSS cmd.arm → 架已接受簽名過的 ARM（架在 ARMED／COUNTDOWN）
  const bool arm = padFresh && (in.padState == FmPad::ARMED || in.padState == FmPad::COUNTDOWN);
  // SSS cmd.fire → 衝突 C-1：點火在架（gate C15）。箭看到「架點火了」的證據就閂：
  //   點火繼電器旗標、架已在 IGNITION 之後，或倒數只剩一幀（臍帶 10 Hz；
  //   避免點火那幀還沒到臍帶就被拔掉，箭永遠看不到點火 → 起飛偵測不開）。
  const bool padIgnited = in.padLinkOk && ((in.padFlags & FmPad::FLAG_IGNITER_ON) != 0 ||
                                           in.padState == FmPad::IGNITION || in.padState == FmPad::RELEASED ||
                                           in.padState == FmPad::LIFTED_OFF);
  const bool padFinalCount = padFresh && in.padState == FmPad::COUNTDOWN && in.padCountdown_s <= m_p.t_fire_lead_s;
  const bool fire = padIgnited || padFinalCount;

  const bool inFlight = inFlightSet(m_s.ds);

  // [FM-04] 地面指令過濾（SSS L207–223）。
  // repo 差異：中止不用密碼，改成「已通過 UplinkAuth（HMAC＋白名單）的 ABORT」＝abortValid（PIL 接）。
  if (inFlight && cmd.override_) addEvent(out, FmEvent::REJECT_OVERRIDE);
  if (cmd.gimbal_ds) addEvent(out, FmEvent::DROP_GIMBAL_DS);
  if (cmd.gimbal_us) addEvent(out, FmEvent::GIMBAL_US);
  if (inFlight && cmd.abortValid) {
    if (!m_s.abort_done) {
      enterAbort(t, out);
    } else {
      addEvent(out, FmEvent::REJECT_ABORT);
    }
  }
  if (cmd.requestMode >= 0 && !legalEdge(m_s.ds, cmd.requestMode)) {
    addEvent(out, FmEvent::REJECT_TRANS);  // 合法請求也不切模式（SSS 同）
  }

  // [FM-05] 假分禁傘（SSS L225–227）：開關已斷、尚未下令分離 → 假分，禁上級傘
  if (in.sep_switch_open && !m_s.sep_fired) m_s.us_chute_inhibit = true;

  // [FM-06] D12 自動中止（SSS L229–252）：三門獨立，一門成立即走
  if (abortableSet(m_s.ds) && m_s.t_liftoff_s >= 0) {
    if (t - m_s.t_liftoff_s >= m_p.t_abort_min_s) {
      m_s.t_tilt_s = (in.tilt_rad > m_p.tilt_abort_rad) ? m_s.t_tilt_s + m_p.dt_s : 0.0f;
      m_s.t_rate_s = (in.rate_rads > m_p.rate_abort_rads) ? m_s.t_rate_s + m_p.dt_s : 0.0f;
      const bool watchdog = (t - m_s.t_liftoff_s) >= m_p.t_watchdog_s;  // 已在 _ABORTABLE 內
      if (m_s.t_tilt_s >= m_p.t_abort_hold_s || m_s.t_rate_s >= m_p.t_abort_hold_s || watchdog) {
        if (!m_s.abort_done) enterAbort(t, out);
      }
    }
  }

  switch (m_s.ds) {
    case FmMode::PAD_IDLE:  // [FM-07]（SSS L255–259）
      if (arm && interlock) setDs(FmMode::ARMED, out);
      if (fire) addEvent(out, FmEvent::DROP_FIRE);  // 未解保險不准點火
      break;

    case FmMode::ARMED:  // [FM-08]（SSS L260–281）
      if (!m_s.fire_latched) {
        if (!interlock) {
          m_s.motor_on = false;
          m_s.t_fire_s = -1;
          setDs(FmMode::PAD_IDLE, out);  // 互鎖掉線（架退 SAFE／臍帶斷）必須重走解保險
          break;
        }
        if (fire) {
          m_s.fire_latched = true;
          m_s.motor_on = true;
          // 只記第一次。倒數證據 → 用架給的剩餘秒數推回 T=0
          m_s.t_fire_s = padIgnited ? t : t + (in.padCountdown_s > 0.0f ? in.padCountdown_s : 0.0f);
        }
      } else {
        // repo 差異：點火後臍帶本來就會拔脫，所以「架狀態變舊」不回 PAD_IDLE（SSS 會）。
        // 只有「架狀態還新」且架明確退回時才動：
        if (in.padLinkOk && in.padState == FmPad::SAFE) {  // 最後一幀前倒數被中止
          m_s.fire_latched = false;
          m_s.motor_on = false;
          m_s.t_fire_s = -1;
          m_s.t_hi_s = 0.0f;
          setDs(FmMode::PAD_IDLE, out);
          break;
        }
        if (in.padLinkOk && padFailed(in.padState)) {  // 架說沒推力／夾住中止／沒離台
          m_s.motor_on = false;
          setDs(FmMode::HANGFIRE, out);
          break;
        }
      }
      if (m_s.fire_latched) {
        m_s.t_hi_s = (in.accel_u_mps2 > m_p.a_liftoff_mps2) ? m_s.t_hi_s + m_p.dt_s : 0.0f;
        if (m_s.t_hi_s >= m_p.n_accel_s) {
          enterLiftoff(t, out);  // 軸向加速度持續滿門檻 → 閂 T0
        } else if (t - m_s.t_fire_s >= m_p.t_hang_s) {
          m_s.motor_on = false;
          setDs(FmMode::HANGFIRE, out);
        }
      }
      break;

    case FmMode::HANGFIRE:  // [FM-08b]（SSS L282–285）地面安全態，不自動重試
      m_s.motor_on = false;
      if (fire) addEvent(out, FmEvent::DROP_FIRE);
      break;

    case FmMode::LIFTOFF:  // [FM-09]（SSS L286–288）進入當拍走的是 ARMED，下一拍才到這
      setDs(FmMode::BOOST_PASSIVE, out);
      break;

    case FmMode::BOOST_PASSIVE: {  // [FM-10]（SSS L289–293）
      const double dtLo = t - m_s.t_liftoff_s;
      const bool ready = in.v_est_mps > m_p.v_active_mps && dtLo >= m_p.t_passive_min_s;
      if (ready || dtLo >= m_p.t_passive_max_s) setDs(FmMode::BOOST_ACTIVE, out);
      break;
    }

    case FmMode::BOOST_ACTIVE:  // [FM-11] SSS _maybe_coast
      maybeCoast(in, t, out);
      break;

    case FmMode::COAST:  // [FM-12] SSS _maybe_sep
      maybeSep(in, t, out);
      break;

    case FmMode::SEPARATED:  // [FM-13]（SSS L298–307）開關斷＝分開 → LS_DESCENT；下令後 5 s 仍通 → SEP_FAIL
      if (in.sep_switch_open) {
        setDs(FmMode::LS_DESCENT, out);
      } else if (m_s.t_sep_cmd_s >= 0 && t - m_s.t_sep_cmd_s >= m_p.t_sep_fail_s) {
        enterSepFail(out);
      }
      break;

    case FmMode::LS_DESCENT:  // [FM-14] SSS _maybe_chute_ds
      maybeChuteDs(in, t, out);
      break;

    case FmMode::LS_CHUTE:  // [FM-15]（SSS L310–315）著地，或離台後 t_land_max 備援
      if (in.contact_ds || (m_s.t_liftoff_s >= 0 && t - m_s.t_liftoff_s >= m_p.t_land_max_s)) {
        m_s.motor_on = false;
        setDs(FmMode::LS_LANDED, out);
      }
      break;

    case FmMode::SEP_FAIL:  // [FM-16]（SSS L316–317）側傘；模式停在 SEP_FAIL
      maybeChuteDs(in, t, out);
      break;

    case FmMode::ABORT:  // [FM-17]（SSS L318–326）中止後分不開 → SEP_FAIL；分得開 → 下級走回收
      if (m_s.t_sep_cmd_s >= 0 && !in.sep_switch_open && t - m_s.t_sep_cmd_s >= m_p.t_sep_fail_s) {
        enterSepFail(out);
      } else if (in.sep_switch_open) {
        setDs(FmMode::LS_DESCENT, out);
      }
      break;

    default:
      break;
  }

  // [FM-19] 組輸出（SSS L331–350）。repo 沒有箭上點火輸出、沒有級間匯流排。
  out.mode = m_s.ds;
  out.authority = authority();
  out.sepFire = m_s.pulse_sep;
  out.sepMask = m_s.pulse_sep ? kFmSepMaskAll : 0;  // 三顆 MG996R 一起（錯開 Δ 由 ActuatorMgr sepStagger_s）
  out.chuteDs = m_s.pulse_chute_ds;
  out.chuteDsSide = m_s.pulse_chute_ds && m_s.ds == FmMode::SEP_FAIL;
  out.chuteUs = false;  // DS 部署不開上級傘
}

void FlightModeCore::enterLiftoff(double t, FmOutputs& out) {
  m_s.t_liftoff_s = t;  // 閂 T0
  m_s.t_hi_s = 0.0f;
  m_s.t_lo_s = 0.0f;
  setDs(FmMode::LIFTOFF, out);
}

// [FM-11] 燃盡：軸向加速度 < a_bo 持續 N ms。時間是窗（T_min 防假燃盡，T_max 防永遠不熄）。
void FlightModeCore::maybeCoast(const FmInputs& in, double t, FmOutputs& out) {
  if (m_s.t_liftoff_s < 0) return;
  const double since = t - m_s.t_liftoff_s;
  if (since < m_p.t_burnout_min_s) {
    m_s.t_lo_s = 0.0f;
    return;
  }
  m_s.t_lo_s = (in.accel_u_mps2 < m_p.a_burnout_mps2) ? m_s.t_lo_s + m_p.dt_s : 0.0f;
  if (m_s.t_lo_s >= m_p.n_burnout_s || since >= m_p.t_burnout_max_s) {
    if (m_s.ds != FmMode::COAST) {
      m_s.t_coast_s = t;  // 分離計時起點
      m_s.t_lo_s = 0.0f;
      setDs(FmMode::COAST, out);
    }
  }
}

// [FM-12] 分離。Edward 2026-11-14：暫定「進 COAST 後計時 4 s 分離」。
// SSS 結構保留：抑制窗 0.8 s ∧（sep_ok ∨ 4 s）；sep_ok 門檻未定（gates D14），輸入恆 false → 實際就是 4 s。
void FlightModeCore::maybeSep(const FmInputs& in, double t, FmOutputs& out) {
  if (m_s.sep_fired || m_s.t_coast_s < 0) return;
  const double waited = t - m_s.t_coast_s;
  if (waited < m_p.t_sep_inhibit_s) return;
  if (in.sep_ok || waited >= m_p.t_sep_backup_s) {
    m_s.sep_fired = true;
    m_s.pulse_sep = true;
    m_s.t_sep_cmd_s = t;
    addEvent(out, FmEvent::CMD_SEP);
    setDs(FmMode::SEPARATED, out);
  }
}

// [FM-14] 下級開傘：分離下令後的軟體計時；accel_high 時抑制；只開一次。
void FlightModeCore::maybeChuteDs(const FmInputs& in, double t, FmOutputs& out) {
  if (m_s.chute_ds_fired || m_s.t_sep_cmd_s < 0) return;
  if (t - m_s.t_sep_cmd_s < m_p.t_chute_ds_s) return;
  if (accelHigh(in)) return;
  m_s.chute_ds_fired = true;
  m_s.pulse_chute_ds = true;
  addEvent(out, FmEvent::CMD_CHUTE_DS);
  if (m_s.ds == FmMode::LS_DESCENT) {
    setDs(FmMode::LS_CHUTE, out);
  } else if (m_s.ds == FmMode::SEP_FAIL) {
    addEvent(out, FmEvent::SIDE_CHUTE);  // 模式維持失敗態
  }
}

// SSS _enter_abort：D12 與已驗證 ABORT 都走這裡。分離＋開傘各只一次。
void FlightModeCore::enterAbort(double t, FmOutputs& out) {
  m_s.abort_done = true;
  m_s.motor_on = false;
  m_s.us_chute_inhibit = true;
  if (!m_s.sep_fired) {
    m_s.sep_fired = true;
    m_s.pulse_sep = true;
    m_s.t_sep_cmd_s = t;
    addEvent(out, FmEvent::CMD_SEP);
  }
  if (!m_s.chute_ds_fired) {
    m_s.chute_ds_fired = true;
    m_s.pulse_chute_ds = true;
    addEvent(out, FmEvent::CMD_CHUTE_DS);
  }
  setDs(FmMode::ABORT, out);
}

// SSS _enter_sep_fail：黏住 → 禁上級傘，下級側傘整箭回收。（DS 部署不推上級模式）
void FlightModeCore::enterSepFail(FmOutputs& out) {
  m_s.us_chute_inhibit = true;
  setDs(FmMode::SEP_FAIL, out);
  addEvent(out, FmEvent::SEP_FAIL);
}

// SSS _advance_us（FM-18 用）
void FlightModeCore::advanceUs(const FmInputs& in, double t, FmOutputs& out) {
  if (m_s.us == FmMode::US_COAST) {
    if (m_s.t_sep_cmd_s >= 0 && t - m_s.t_sep_cmd_s >= m_p.t_chute_us_s && !accelHigh(in) &&
        !m_s.us_chute_inhibit && !m_s.chute_us_fired) {
      m_s.chute_us_fired = true;
      m_s.pulse_chute_us = true;
      setUs(FmMode::US_CHUTE, out);
      addEvent(out, FmEvent::CMD_CHUTE_US);
    }
    if (in.contact_us) setUs(FmMode::US_LANDED, out);
  } else if (m_s.us == FmMode::US_CHUTE) {
    if (in.contact_us || (m_s.t_liftoff_s >= 0 && t - m_s.t_liftoff_s >= m_p.t_land_max_s)) {
      setUs(FmMode::US_LANDED, out);
    }
  }
}

// [FM-18] 上級部署（SSS _tick_us L478–518）。
// repo 差異：上下級沒有通訊（Edward 2026-11-14）。SSS 從級間鏈路收的 sep_cmd／t_sep_cmd／us_inhibit 都沒有，
// 上級唯一的分離證據是自己的分離開關：開關斷 → 當作已分離，開傘計時從「開關斷」起算（SSS 從下級下令起算，
// 差一個舵機放開的時間）。
// ⚠️ 問題表 P-1：沒有鏈路，上級分不出「墊上被碰開（假分）」和「真的分離」，墊上碰開會在 3.2 s 後開上級傘。
//    硬體要解（例如分離開關要有機械保險到離軌才解除，或上級加 IMU 自己確認離台），見 README。
void FlightModeCore::tickUs(const FmInputs& in, const FmGroundCmd& cmd, FmOutputs& out) {
  const double t = in.t_s;
  m_s.pulse_sep = false;
  m_s.pulse_chute_ds = false;
  m_s.pulse_chute_us = false;
  if (cmd.gimbal_us) addEvent(out, FmEvent::GIMBAL_US);
  if (m_s.us == FmMode::US_WAIT) {
    if (m_s.us_chute_inhibit) {
      setUs(FmMode::US_CHUTE_INHIBIT, out);
    } else if (in.sep_switch_open) {
      m_s.sep_fired = true;
      m_s.t_sep_cmd_s = t;
      setUs(FmMode::US_COAST, out);
    }
  } else if (m_s.us == FmMode::US_COAST || m_s.us == FmMode::US_CHUTE) {
    advanceUs(in, t, out);
  }
  out.mode = m_s.us;
  out.authority = FmAuthority::CENTER;
  out.chuteUs = m_s.pulse_chute_us;
}

// SSS _authority（L528–542）
FmAuthority FlightModeCore::authority() const {
  if (m_s.imu_stale_s >= m_p.t_imu_stale_s) return FmAuthority::CENTER;
  switch (m_s.ds) {
    case FmMode::SEPARATED:
    case FmMode::LS_DESCENT:
    case FmMode::LS_CHUTE:
    case FmMode::LS_LANDED:
    case FmMode::SEP_FAIL:
    case FmMode::ABORT:
      return FmAuthority::LOCKED_CENTER;
    case FmMode::HANGFIRE:
      return FmAuthority::CENTER;
    case FmMode::BOOST_ACTIVE:
    case FmMode::COAST:
      return m_s.abort_done ? FmAuthority::LOCKED_CENTER : FmAuthority::CLOSED_LOOP;
    default:
      return FmAuthority::CENTER;
  }
}

}  // namespace Mcu
