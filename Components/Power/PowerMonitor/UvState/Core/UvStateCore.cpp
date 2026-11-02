#include "UvStateCore.hpp"

namespace Components {
namespace UvState {

UvStateCore::UvStateCore()
    : m_cfg(),
      m_level(NOMINAL),
      m_dwell_accum_s(0.0f),
      m_last_time_s(0.0f),
      m_have_last_time(false),
      m_rejected_count(0),
      m_transition_count(0) {
    // Placeholder defaults; real values come from prmDb.
    m_cfg.warn_entry_V = 0.0f;
    m_cfg.shed_entry_V = 0.0f;
    m_cfg.critical_entry_V = 0.0f;
    m_cfg.warn_dwell_s = 0.0f;
    m_cfg.shed_dwell_s = 0.0f;
    m_cfg.critical_dwell_s = 0.0f;
    m_cfg.warn_hyst_V = 0.0f;
    m_cfg.shed_hyst_V = 0.0f;
    m_cfg.critical_hyst_V = 0.0f;
    m_cfg.max_sample_age_s = 0.0f;
}

void UvStateCore::configure(const UvConfig& cfg) {
    m_cfg = cfg;
}

void UvStateCore::reset() {
    m_level = NOMINAL;
    m_dwell_accum_s = 0.0f;
    m_last_time_s = 0.0f;
    m_have_last_time = false;
    m_rejected_count = 0;
    m_transition_count = 0;
}

UvLevel UvStateCore::classify(float voltage_V) const {
    // Lower voltage => more severe. Check most severe first.
    if (voltage_V < m_cfg.critical_entry_V) {
        return UV_CRITICAL;
    }
    if (voltage_V < m_cfg.shed_entry_V) {
        return UV_SHED;
    }
    if (voltage_V < m_cfg.warn_entry_V) {
        return UV_WARN;
    }
    return NOMINAL;
}

bool UvStateCore::may_exit(UvLevel current, float voltage_V) const {
    // To exit a level, voltage must rise above entry + hysteresis.
    switch (current) {
        case UV_CRITICAL:
            return voltage_V > (m_cfg.critical_entry_V + m_cfg.critical_hyst_V);
        case UV_SHED:
            return voltage_V > (m_cfg.shed_entry_V + m_cfg.shed_hyst_V);
        case UV_WARN:
            return voltage_V > (m_cfg.warn_entry_V + m_cfg.warn_hyst_V);
        case NOMINAL:
        default:
            return true;
    }
}

float UvStateCore::dwell_required(UvLevel target) const {
    switch (target) {
        case UV_CRITICAL:
            return m_cfg.critical_dwell_s;
        case UV_SHED:
            return m_cfg.shed_dwell_s;
        case UV_WARN:
            return m_cfg.warn_dwell_s;
        case NOMINAL:
        default:
            return 0.0f;
    }
}

UvOutputs UvStateCore::make_transition(UvLevel old_level, UvLevel new_level) {
    m_level = new_level;
    m_dwell_accum_s = 0.0f;
    m_transition_count++;

    UvOutputs out;
    out.level_changed = true;
    out.old_level = old_level;
    out.new_level = new_level;
    out.sample_rejected = false;

    // Emit load-shed requests based on the new level.
    // L1: camera, gimbal, humidity sensor off.
    // L2: high-rate logger off, reduce telemetry rate.
    // No L3 / tail-fin lock from the power group.
    switch (new_level) {
        case UV_CRITICAL:
            out.emit_l1 = true;
            out.emit_l2 = true;
            break;
        case UV_SHED:
            out.emit_l1 = true;
            out.emit_l2 = false;
            break;
        case UV_WARN:
        case NOMINAL:
        default:
            out.emit_l1 = false;
            out.emit_l2 = false;
            break;
    }
    return out;
}

UvOutputs UvStateCore::update(const PowerRailTlm& sample, float now_s) {
    UvOutputs out;
    out.emit_l1 = false;
    out.emit_l2 = false;
    out.level_changed = false;
    out.new_level = m_level;
    out.old_level = m_level;
    out.sample_rejected = false;

    // Compute elapsed time since last update (injected time only).
    float dt = 0.0f;
    if (m_have_last_time) {
        dt = now_s - m_last_time_s;
        if (dt < 0.0f) {
            dt = 0.0f;  // guard against non-monotonic injection
        }
    }
    m_last_time_s = now_s;
    m_have_last_time = true;

    // Reject invalid or stale samples: they do NOT count toward dwell.
    bool usable = sample.valid;
    if (usable && m_cfg.max_sample_age_s > 0.0f &&
        sample.sample_age_s > m_cfg.max_sample_age_s) {
        usable = false;
    }
    if (!usable) {
        out.sample_rejected = true;
        m_rejected_count++;
        // Do not accumulate dwell on rejected samples.
        return out;
    }

    // Determine target level from voltage.
    UvLevel target = classify(sample.voltage_V);

    if (target == m_level) {
        // Same level: reset dwell accumulator (condition not pending).
        m_dwell_accum_s = 0.0f;
        return out;
    }

    // Exiting to a less severe level: hysteresis gate, then immediate.
    if (target < m_level) {
        if (!may_exit(m_level, sample.voltage_V)) {
            m_dwell_accum_s = 0.0f;
            return out;
        }
        // Step down one level at a time when intermediate thresholds apply.
        UvLevel step = static_cast<UvLevel>(static_cast<uint8_t>(m_level) - 1u);
        // If voltage already classifies even lower severity, jump to target.
        if (target < step) {
            // Still require may_exit for each level we leave; already passed
            // for current. Jumping directly to target is OK once hysteresis
            // for the current level is met and classify says target.
            step = target;
        }
        return make_transition(m_level, step);
    }

    // Entering a more severe level: require dwell persistence.
    m_dwell_accum_s += dt;
    float required = dwell_required(target);
    if (m_dwell_accum_s < required) {
        return out;
    }

    return make_transition(m_level, target);
}

}  // namespace UvState
}  // namespace Components
