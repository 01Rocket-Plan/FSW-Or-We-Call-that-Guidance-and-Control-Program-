#include "PadControllerCore.hpp"

#include <cmath>

namespace PadLink {

namespace {

bool isTerminal(PadState s) {
  return s == PadState::LIFTED_OFF || s == PadState::HANG_FIRE || s == PadState::HOLD_ABORT ||
         s == PadState::NO_LIFTOFF;
}

std::uint16_t clampU16(double v) {
  if (!(v > 0.0)) return 0;  // 也擋掉 NaN
  if (v > 65535.0) return 65535u;
  return static_cast<std::uint16_t>(v + 0.5);
}

bool timeParamOk(double v) { return std::isfinite(v) && v >= 0.0; }

}  // namespace

PadControllerCore::PadControllerCore(IPadCmdAuth* auth, std::uint8_t bootId,
                                     IPadEvents* events)
    : m_gate(auth, bootId), m_events(events) {
  m_paramsOk = paramsValid(m_p);
}

bool PadControllerCore::paramsValid(const PadParams& p) {
  const double t[] = {p.countdown_s,         p.ignitionWindow_s,       p.thrustConfirm_s,
                      p.releaseMargin_s,     p.clampActuatorLatency_s, p.liftoffTimeout_s,
                      p.releaseDeadline_s,   p.groundLinkTimeout_s,    p.umbilicalTimeout_s,
                      p.liftoffLoadHold_s};
  for (double v : t) {
    if (!timeParamOk(v)) return false;
  }
  if (!(p.thrustConfirm_s > 0.0) || !(p.liftoffTimeout_s > 0.0)) return false;
  if (!std::isfinite(p.clampOpenTimeout_s) || !std::isfinite(p.liftoffLoadMin_N)) return false;
  // 鬆夾持整段時序（確認＋裕度＋夾持動作）必須在期限內。
  return p.thrustConfirm_s + p.releaseMargin_s + p.clampActuatorLatency_s <=
         p.releaseDeadline_s + kTimeEps_s;
}

bool PadControllerCore::configure(const PadParams& p) {
  m_p = p;
  m_paramsOk = paramsValid(p);
  if (!m_paramsOk && m_events != nullptr) m_events->onBadParams(p);
  return m_paramsOk;
}

void PadControllerCore::setState(PadState s, PadReason r) {
  const PadState from = m_state;
  m_reason = r;
  if (from == s) return;
  m_state = s;
  if (m_events != nullptr) m_events->onStateChange(from, s, r);
}

std::uint16_t PadControllerCore::computeInterlocks(const PadInputs& in, double now_s) {
  m_rocketFresh = m_haveRocket && (now_s - m_lastRocketRx_s) <= m_p.umbilicalTimeout_s;
  const bool groundFresh =
      m_haveGround && (now_s - m_lastGroundRx_s) <= m_p.groundLinkTimeout_s;
  std::uint16_t il = 0;
  if (in.seated) il |= IL_SEATED;
  if (in.continuity) il |= IL_CONTINUITY;
  if (in.keyOn) il |= IL_KEY_ON;
  if (in.tilt_rad >= 0.0 && in.tilt_rad <= m_p.maxTilt_rad) il |= IL_TILT_OK;  // NaN → 紅
  if (!in.railJam) il |= IL_RAIL_FREE;
  if (in.padV_V >= m_p.padVmin_V) il |= IL_PAD_POWER_OK;
  if (groundFresh) il |= IL_GROUND_LINK_OK;
  if (m_rocketFresh) il |= IL_ROCKET_LINK_OK;
  if (m_rocketFresh && m_rocket.ready) il |= IL_ROCKET_READY;
  if (m_state == PadState::ARMED || m_state == PadState::COUNTDOWN) il |= IL_SW_ARMED;
  const bool clampReleased = m_state == PadState::RELEASED || m_state == PadState::LIFTED_OFF ||
                             m_state == PadState::NO_LIFTOFF;
  if (!clampReleased) il |= IL_CLAMP_LOCKED;
  return il;
}

PadReason PadControllerCore::firstRed(std::uint16_t il) {
  if (!(il & IL_SEATED)) return PadReason::NOT_SEATED;
  if (!(il & IL_CONTINUITY)) return PadReason::NO_CONTINUITY;
  if (!(il & IL_KEY_ON)) return PadReason::KEY_OFF;
  if (!(il & IL_TILT_OK)) return PadReason::TILT;
  if (!(il & IL_RAIL_FREE)) return PadReason::RAIL_JAM;
  if (!(il & IL_PAD_POWER_OK)) return PadReason::PAD_POWER;
  if (!(il & IL_GROUND_LINK_OK)) return PadReason::GROUND_LINK_LOST;
  if (!(il & IL_ROCKET_LINK_OK)) return PadReason::ROCKET_LINK_LOST;
  if (!(il & IL_ROCKET_READY)) return PadReason::ROCKET_NOT_READY;
  return PadReason::NONE;
}

void PadControllerCore::onUmbilicalBytes(const std::uint8_t* data, std::size_t len,
                                         double now_s) {
  if (data == nullptr) return;
  DecodedFrame f;
  for (std::size_t i = 0; i < len; ++i) {
    if (!m_deframer.push(data[i], f)) continue;
    if (f.type != PadMsgType::ROCKET_STATUS) continue;  // 臍帶上只收箭體狀態
    RocketStatusMsg m;
    if (!unpackRocketStatus(f.payload, f.len, m)) continue;
    m_rocket = m;
    m_haveRocket = true;
    m_lastRocketRx_s = now_s;
  }
}

PadCmdResult PadControllerCore::onGroundPacket(const std::uint8_t* data, std::size_t len,
                                               double now_s) {
  DecodedFrame f;
  GroundCmdMsg cmd;
  if (!decodeFrame(data, len, f) || f.type != PadMsgType::GROUND_CMD ||
      !unpackGroundCmd(f.payload, f.len, cmd)) {
    ++m_groundRejects;
    return PadCmdResult::BAD_FRAME;
  }
  const PadCmdResult gate = m_gate.check(cmd);
  PadReason why = PadReason::NONE;
  PadCmdResult res = gate;
  if (gate == PadCmdResult::ACCEPTED) {
    if (PadCmdGate::requiresAuth(cmd.op)) {
      // 只有簽過名的幀才算地面鏈路活著。
      m_haveGround = true;
      m_lastGroundRx_s = now_s;
      m_lastCmdCounter = cmd.counter;
    }
    res = handleCommand(cmd, now_s, why);
  } else {
    ++m_groundRejects;
  }
  m_lastCmdResult = res;
  if (m_events != nullptr) m_events->onCommand(cmd.op, res, why);
  return res;
}

PadCmdResult PadControllerCore::handleCommand(const GroundCmdMsg& cmd, double now_s,
                                              PadReason& why) {
  m_il = computeInterlocks(m_lastIn, now_s);
  const bool allGreen = (m_il & kPadPermitMask) == kPadPermitMask;
  switch (cmd.op) {
    case PadCmdOp::HEARTBEAT:
      return PadCmdResult::ACCEPTED;

    case PadCmdOp::ARM:
      if (m_ignitionUsed) {
        why = PadReason::IGNITION_LOCKED;
        return PadCmdResult::REJECTED;
      }
      if (m_state != PadState::SAFE) {
        why = PadReason::BUSY;
        return PadCmdResult::REJECTED;
      }
      if (!m_paramsOk) {
        why = PadReason::BAD_PARAMS;
        m_reason = why;
        return PadCmdResult::REJECTED;
      }
      if (!allGreen) {
        why = firstRed(m_il);
        m_reason = why;
        return PadCmdResult::REJECTED;
      }
      setState(PadState::ARMED, PadReason::NONE);
      return PadCmdResult::ACCEPTED;

    case PadCmdOp::LAUNCH:
      if (m_state != PadState::ARMED) {
        why = PadReason::NOT_ARMED;  // 兩道指令必須分開，且要先 ARM
        return PadCmdResult::REJECTED;
      }
      if (!allGreen) {
        why = firstRed(m_il);
        setState(PadState::SAFE, why);
        return PadCmdResult::REJECTED;
      }
      m_tCountdown_s = now_s;
      m_countdownRemaining_s = m_p.countdown_s;
      setState(PadState::COUNTDOWN, PadReason::NONE);
      return PadCmdResult::ACCEPTED;

    case PadCmdOp::ABORT:
      if (m_state == PadState::ARMED || m_state == PadState::COUNTDOWN) {
        setState(PadState::SAFE, PadReason::CMD_ABORT);
        return PadCmdResult::ACCEPTED;
      }
      if (m_state == PadState::IGNITION) {
        // 夾持還沒鬆 → 點火關、夾持保持鎖死（H2）。
        setState(PadState::HOLD_ABORT, PadReason::CMD_ABORT);
        return PadCmdResult::ACCEPTED;
      }
      if (m_state == PadState::RELEASED) {
        why = PadReason::BUSY;  // 已鬆夾持，不能再夾回（H2）；離軌後由箭上 A5 處理
        return PadCmdResult::REJECTED;
      }
      return PadCmdResult::ACCEPTED;  // SAFE／終態：本來就安全

    case PadCmdOp::DISARM:
      if (m_state == PadState::ARMED || m_state == PadState::COUNTDOWN) {
        setState(PadState::SAFE, PadReason::CMD_DISARM);
        return PadCmdResult::ACCEPTED;
      }
      if (m_state == PadState::SAFE || isTerminal(m_state)) return PadCmdResult::ACCEPTED;
      why = PadReason::BUSY;  // IGNITION／RELEASED 用 ABORT，不是 DISARM
      return PadCmdResult::REJECTED;
  }
  why = PadReason::NONE;
  return PadCmdResult::BAD_FRAME;
}

PadOutputs PadControllerCore::tick(const PadInputs& in, double now_s) {
  m_lastIn = in;
  m_il = computeInterlocks(in, now_s);
  const bool allGreen = (m_il & kPadPermitMask) == kPadPermitMask;

  switch (m_state) {
    case PadState::SAFE:
      break;

    case PadState::ARMED:
      if (!m_paramsOk) setState(PadState::SAFE, PadReason::BAD_PARAMS);
      else if (!allGreen) setState(PadState::SAFE, firstRed(m_il));
      break;

    case PadState::COUNTDOWN:
      if (!allGreen || !m_paramsOk) {
        // 一紅就終止倒數；要重新完整走 ARM → LAUNCH，不會自動恢復。
        setState(PadState::SAFE, m_paramsOk ? firstRed(m_il) : PadReason::BAD_PARAMS);
        m_countdownRemaining_s = 0.0;
        break;
      }
      m_countdownRemaining_s = m_p.countdown_s - (now_s - m_tCountdown_s);
      if (m_countdownRemaining_s <= 0.0) {
        m_countdownRemaining_s = 0.0;
        m_ignitionUsed = true;  // 一次開機只點一次
        m_tIgnition_s = now_s;
        m_thrustSince_s = -1.0;
        m_thrustFirst_s = -1.0;
        m_tConfirm_s = -1.0;
        setState(PadState::IGNITION, PadReason::NONE);
      }
      break;

    case PadState::IGNITION: {
      // 夾持還沒鬆：異常就點火關、保持鎖死。地面斷線不算異常（點了就只能自己走完）；
      // 點火迴路連續性在引線燒斷後本來就會開路，也不算。
      PadReason bad = PadReason::NONE;
      if (!in.keyOn) bad = PadReason::KEY_OFF;
      else if (!(m_il & IL_PAD_POWER_OK)) bad = PadReason::PAD_POWER;
      else if (!m_rocketFresh) bad = PadReason::ROCKET_LINK_LOST;  // 臍帶此時應該還插著
      else if (in.railJam) bad = PadReason::RAIL_JAM;
      if (bad != PadReason::NONE) {
        setState(PadState::HOLD_ABORT, bad);
        break;
      }
      // 鬆夾持期限（Edward PR #52 審查）：第一次看到推力後太久還沒鬆 → 不鬆、繼續夾住。
      const double deadline =
          m_p.releaseDeadline_s > m_p.thrustConfirm_s ? m_p.releaseDeadline_s : m_p.thrustConfirm_s;
      if (m_tConfirm_s >= 0.0) {
        // 推力已確認，等 releaseMargin_s：這段時間推力只要消失一拍就不鬆（鬆架那一拍也要有推力）。
        if (!in.thrustOk) {
          setState(PadState::HOLD_ABORT, PadReason::THRUST_LOST);
        } else if (now_s - m_thrustFirst_s > deadline + kTimeEps_s) {
          setState(PadState::HOLD_ABORT, PadReason::LATE_RELEASE);
        } else if (now_s - m_tConfirm_s + kTimeEps_s >= m_p.releaseMargin_s) {
          m_tRelease_s = now_s;
          m_clampOpenSeen = false;
          m_loadLowSince_s = -1.0;
          setState(PadState::RELEASED, PadReason::NONE);
        }
        break;
      }
      if (in.thrustOk) {
        if (m_thrustFirst_s < 0.0) m_thrustFirst_s = now_s;
        if (m_thrustSince_s < 0.0) m_thrustSince_s = now_s;
      } else {
        m_thrustSince_s = -1.0;
      }
      if (m_thrustFirst_s >= 0.0) {
        if (now_s - m_thrustFirst_s > deadline + kTimeEps_s) {
          setState(PadState::HOLD_ABORT, PadReason::LATE_RELEASE);
          break;
        }
        if (in.thrustOk && now_s - m_thrustSince_s + kTimeEps_s >= m_p.thrustConfirm_s) {
          m_tConfirm_s = now_s;
          if (m_p.releaseMargin_s <= kTimeEps_s) {  // 裕度 0：確認那拍就鬆（舊行為）
            m_tRelease_s = now_s;
            m_clampOpenSeen = false;
            m_loadLowSince_s = -1.0;
            setState(PadState::RELEASED, PadReason::NONE);
          }
          break;
        }
      }
      if (now_s - m_tIgnition_s >= m_p.ignitionWindow_s) {
        setState(PadState::HANG_FIRE, PadReason::NO_THRUST);  // 不自動重試
      }
      break;
    }

    case PadState::RELEASED: {
      // 離軌判定三層（Edward PR #52 審查）：
      //   主判定 offRail 單獨成立就算（夾持開到位開關壞掉也照算）；
      //   備援 荷重元低於 liftoffLoadMin_N 持續 liftoffLoadHold_s；
      //   輔助 臍帶逾時單獨不算（可能只是被震斷），要和荷重元低同時成立才算。
      //   備援／輔助都要先讀到夾持開到位（沒裝開關時不等）：夾持沒開，荷重元掉下來不代表箭走了。
      const bool clampCheck = m_p.clampOpenTimeout_s > 0.0;
      if (in.clampOpen) m_clampOpenSeen = true;
      const bool loadLow = in.load_N < m_p.liftoffLoadMin_N;  // NaN → false（不算離軌）
      if (loadLow) {
        if (m_loadLowSince_s < 0.0) m_loadLowSince_s = now_s;
      } else {
        m_loadLowSince_s = -1.0;
      }
      const bool loadHeld =
          loadLow && now_s - m_loadLowSince_s + kTimeEps_s >= m_p.liftoffLoadHold_s;
      const bool backupAllowed = !clampCheck || m_clampOpenSeen;
      if (in.offRail || (backupAllowed && (loadHeld || (loadLow && !m_rocketFresh)))) {
        setState(PadState::LIFTED_OFF, PadReason::NONE);
      } else if (clampCheck && !m_clampOpenSeen &&
                 now_s - m_tRelease_s + kTimeEps_s >= m_p.clampOpenTimeout_s) {
        // 夾持卡住：點火早已關、鬆夾持輸出維持 true（不會再夾回），通知人員不要靠近。
        setState(PadState::NO_LIFTOFF, PadReason::CLAMP_STUCK);
      } else if (now_s - m_tRelease_s >= m_p.liftoffTimeout_s) {
        setState(PadState::NO_LIFTOFF, PadReason::NO_LIFTOFF);
      }
      break;
    }

    case PadState::LIFTED_OFF:
    case PadState::HANG_FIRE:
    case PadState::HOLD_ABORT:
    case PadState::NO_LIFTOFF:
      break;  // 終態：不重試、不自動回 SAFE
  }

  m_il = computeInterlocks(in, now_s);  // 狀態可能變了（SW_ARMED／CLAMP 位元）
  PadOutputs out;
  out.igniterOn = (m_state == PadState::IGNITION) && in.keyOn;
  out.clampRelease = m_state == PadState::RELEASED || m_state == PadState::LIFTED_OFF ||
                     m_state == PadState::NO_LIFTOFF;
  m_lastOut = out;
  return out;
}

std::size_t PadControllerCore::buildStatusFrame(std::uint8_t* out, std::size_t cap) {
  PadStatusMsg m;
  m.padState = static_cast<std::uint8_t>(m_state);
  m.lastReason = static_cast<std::uint8_t>(m_reason);
  m.interlocks = m_il;
  m.rocketMode = m_rocket.flightMode;
  m.rocketBit = m_rocketFresh ? m_rocket.bitFlags : 0u;
  m.tilt_cdeg = clampU16(m_lastIn.tilt_rad * (18000.0 / 3.14159265358979323846));
  m.padV_mV = clampU16(m_lastIn.padV_V * 1000.0);
  m.countdown_ds = clampU16(m_countdownRemaining_s * 10.0);
  m.lastCmdCounter = m_lastCmdCounter;
  m.lastCmdResult = static_cast<std::uint8_t>(m_lastCmdResult);
  m.bootId = m_gate.bootId();
  std::uint8_t payload[kPadStatusBytes];
  if (packPadStatus(m, payload, sizeof(payload)) == 0) return 0;
  return encodeFrame(PadMsgType::PAD_STATUS, m_statusSeq++, payload, sizeof(payload), out, cap);
}

std::size_t PadControllerCore::buildUmbilicalFrame(std::uint8_t* out, std::size_t cap) {
  PadToRocketMsg m;
  m.padState = static_cast<std::uint8_t>(m_state);
  m.lastReason = static_cast<std::uint8_t>(m_reason);
  m.countdown_ds = clampU16(m_countdownRemaining_s * 10.0);
  std::uint8_t f = 0;
  if (m_lastOut.igniterOn) f |= PTR_IGNITER_ON;
  if (m_state == PadState::RELEASED || m_state == PadState::LIFTED_OFF ||
      m_state == PadState::NO_LIFTOFF) {
    f |= PTR_THRUST_CONFIRMED;  // 只有推力確認後才會進 RELEASED
  }
  if (m_lastOut.clampRelease) f |= PTR_CLAMP_RELEASED;
  if (m_rocketFresh) f |= PTR_ROCKET_LINK_OK;
  m.flags = f;
  m.padBootId = m_gate.bootId();
  std::uint8_t payload[kPadToRocketBytes];
  if (packPadToRocket(m, payload, sizeof(payload)) == 0) return 0;
  return encodeFrame(PadMsgType::PAD_TO_ROCKET, m_umbilicalSeq++, payload, sizeof(payload), out,
                     cap);
}

}  // namespace PadLink
