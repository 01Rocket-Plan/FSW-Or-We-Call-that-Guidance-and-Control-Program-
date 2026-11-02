#include "ServoSimCore.hpp"

namespace ServoSim {

// ======== ServoSimCore ========
void ServoSimCore::enter(DrvState s) { m_state = s; m_stateTicks = 0; }

void ServoSimCore::allNeutral() { for (uint8_t i = 0; i < m_p.channel_count; i++) m_pulse[i] = m_p.neutral_us; }

void ServoSimCore::emit() { for (uint8_t i = 0; i < m_p.channel_count; i++) m_sink.pwmOut(i, m_pulse[i]); }

void ServoSimCore::start() { if (m_state == DrvState::OFF) { allNeutral(); enter(DrvState::INIT); } }

void ServoSimCore::tick() {
  m_stateTicks++;
  switch (m_state) {
    case DrvState::OFF: return;  // 未上電：不輸出
    case DrvState::INIT: enter(DrvState::SELFTEST); break;
    case DrvState::SELFTEST: {
      const bool idOk =
          (m_p.model == ServoModel::SG90 && m_p.role == ServoRole::TAIL && m_p.channel_count == 4) ||
          (m_p.model == ServoModel::SG90 && m_p.role == ServoRole::GIMBAL && m_p.channel_count == 2);
      if (!idOk) {
        enter(DrvState::FAULT);
        m_sink.evModelRoleMismatch(m_p.model, m_p.role, m_p.channel_count);
        m_sink.evEnterFault(FaultReason::MODEL_ROLE_MISMATCH);
        break;
      }
      bool fail = false;
      if (m_hw.selfTestDone(fail)) { enter(DrvState::READY); m_ageTicks = 0; m_timedOut = false; m_sink.evEnterReady(); }
      else if (fail) { enter(DrvState::FAULT); m_sink.evEnterFault(FaultReason::SELFTEST_FAIL); }
      else if (m_stateTicks * (1.0f / m_p.pwm_hz) >= m_p.selftest_deadline_s) {
        enter(DrvState::FAULT); m_sink.evEnterFault(FaultReason::SELFTEST_TIMEOUT);
      }
      break;
    }
    case DrvState::READY:
      m_ageTicks++;
      if (!m_timedOut && cmdAge_s() > m_p.cmd_timeout_s + 1e-6f) {
        m_timedOut = true; allNeutral();
        m_sink.evCmdTimeout(cmdAge_s()); m_sink.evLockNeutral(NeutralSource::TIMEOUT);
      }
      break;
    case DrvState::FAULT: allNeutral(); break;  // FAULT：保持中立
  }
  emit();  // 零階保持：每 tick 輸出目前值
}

void ServoSimCore::cmdIn(const ServoCmd& c) {
  if (m_state != DrvState::READY) { m_drop++; m_sink.evCmdDropped(c.seq, DropReason::NOT_READY); return; }
  if (m_haveSeq && c.seq == m_lastSeq) { m_drop++; m_sink.evCmdDropped(c.seq, DropReason::SEQ_DUP); return; }
  if (m_haveSeq && c.seq < m_lastSeq) { m_drop++; m_sink.evCmdDropped(c.seq, DropReason::SEQ_BACKWARD); return; }
  m_lastSeq = c.seq; m_haveSeq = true; m_ageTicks = 0; m_timedOut = false;
  if (c.lockNeutral) { allNeutral(); m_sink.evLockNeutral(NeutralSource::CMD); return; }
  for (uint8_t i = 0; i < m_p.channel_count; i++) {  // channel_count=2 時 [2]、[3] 忽略
    uint16_t raw = c.pulse_us[i], used = raw;
    if (used < m_p.pulse_min_us) used = m_p.pulse_min_us;
    if (used > m_p.pulse_max_us) used = m_p.pulse_max_us;
    if (used != raw) { m_clamp++; m_sink.evPulseClamped(i, raw, used); }
    m_pulse[i] = used;
  }
}

void ServoSimCore::uvIn(UvState st) { m_uv = st; m_sink.evUvSeen(st); }  // 只記錄，不動舵

// ======== SepSimCore ========
uint16_t SepSimCore::clampUs(uint16_t v) const {
  return v < m_p.pulse_min_us ? m_p.pulse_min_us : (v > m_p.pulse_max_us ? m_p.pulse_max_us : v);
}

void SepSimCore::enter(DrvState s, FaultReason r) {
  m_state = s; m_stateTicks = 0;
  if (s == DrvState::FAULT) { m_sink.evEnterFault(r); m_sink.evSepFaultHold(lockedMask()); }
}

void SepSimCore::emit() { for (uint8_t i = 0; i < 4; i++) m_sink.pwmOut(i, m_pulse[i]); }

void SepSimCore::start() {
  if (m_state != DrvState::OFF) return;
  for (auto& v : m_pulse) v = clampUs(m_p.lock_us);
  m_state = DrvState::INIT; m_stateTicks = 0;
}

void SepSimCore::tick() {
  m_stateTicks++;
  switch (m_state) {
    case DrvState::OFF: return;
    case DrvState::INIT: {
      uint8_t mask = 0;
      if (!m_nv.sepReleasedGet(mask)) { enter(DrvState::FAULT, FaultReason::NV_READ_FAIL); break; }
      m_fired = mask & 0x0F;  // 開機時已釋放的通道：不再輸出 fire_us，維持 lock_us（對齊真驅動）
      enter(DrvState::SELFTEST);
      break;
    }
    case DrvState::SELFTEST: {
      if (!(m_p.model == ServoModel::MG996R && m_p.role == ServoRole::SEP)) {
        m_sink.evModelRoleMismatch(m_p.model, m_p.role);
        enter(DrvState::FAULT, FaultReason::MODEL_ROLE_MISMATCH);
        break;
      }
      bool fail = false;
      if (m_hw.selfTestDone(fail)) { enter(DrvState::READY); m_sink.evEnterReady(); }
      else if (fail) enter(DrvState::FAULT, FaultReason::SELFTEST_FAIL);
      else if (m_stateTicks / m_p.tick_hz >= m_p.selftest_deadline_s) enter(DrvState::FAULT, FaultReason::SELFTEST_TIMEOUT);
      break;
    }
    case DrvState::READY:
      for (uint8_t i = 0; i < 4; i++)
        if (m_holdTicks[i] > 0) --m_holdTicks[i];  // 保持結束後仍停在 fire_us（已釋放通道保持釋放，對齊真驅動）
      break;
    case DrvState::FAULT: break;  // 維持現狀，SEP 不動
  }
  emit();
}

void SepSimCore::cmdIn(const SepCmd& c) {
  if (m_state != DrvState::READY) { m_sink.evCmdDropped(c.seq, DropReason::NOT_READY); return; }
  if (!c.valid) { m_sink.evCmdDropped(c.seq, DropReason::INVALID); return; }
  if (m_haveSeq && c.seq == m_lastSeq) { m_sink.evCmdDropped(c.seq, DropReason::SEQ_DUP); return; }
  if (m_haveSeq && c.seq < m_lastSeq) { m_sink.evCmdDropped(c.seq, DropReason::SEQ_BACKWARD); return; }
  m_lastSeq = c.seq; m_haveSeq = true;
  for (uint8_t ch = 0; ch < 4; ch++) {
    const uint8_t bit = static_cast<uint8_t>(1u << ch);
    if (!(c.channelMask & bit)) continue;
    uint8_t nvMask = 0;
    if (!m_nv.sepReleasedGet(nvMask)) { m_sink.evSepNvWriteFail(ch); continue; }  // 讀失敗也不釋放
    if ((nvMask | m_fired) & bit) { m_fired |= bit; m_sink.evSepAlreadyFired(ch); continue; }
    if (!m_nv.sepReleasedSet(bit)) { m_sink.evSepNvWriteFail(ch); continue; }  // 維持 lock_us，下次可重試
    m_fired |= bit;
    m_pulse[ch] = clampUs(m_p.fire_us);
    uint32_t hold = static_cast<uint32_t>(m_p.fire_hold_s * m_p.tick_hz + 0.5f);
    m_holdTicks[ch] = hold ? hold : 1;
    m_sink.evSepFire(ch);
  }
}

void SepSimCore::uvIn(UvState st) { m_uv = st; m_sink.evUvSeen(st); }

}  // namespace ServoSim
