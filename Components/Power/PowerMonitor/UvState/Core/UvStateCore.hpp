#ifndef COMPONENTS_UVSTATE_CORE_UVSTATECORE_HPP
#define COMPONENTS_UVSTATE_CORE_UVSTATECORE_HPP

#include <cstdint>

namespace Components {
namespace UvState {

// NOTE: The FPP enum is named UvState (reserved in RocketTypes). In the core
// we use UvLevel to avoid clashing with the FPP-generated enum type name.
enum UvLevel : uint8_t {
    NOMINAL = 0,
    UV_WARN = 1,
    UV_SHED = 2,
    UV_CRITICAL = 3
};

// Minimal power rail telemetry sample. No dynamic allocation.
struct PowerRailTlm {
    uint32_t rail_id;      // identifier of the rail
    float voltage_V;       // measured voltage in volts
    float current_A;       // measured current in amps
    float sample_age_s;    // age of the sample in seconds
    bool valid;            // false => sample must NOT count toward dwell
};

// Per-level configuration. All values are placeholders; real numbers come
// from prmDb at runtime. Do NOT treat these as authoritative thresholds.
struct UvConfig {
    // Entry thresholds (voltage below which the level is entered).
    float warn_entry_V;
    float shed_entry_V;
    float critical_entry_V;

    // Dwell times (seconds a condition must persist before transition).
    float warn_dwell_s;
    float shed_dwell_s;
    float critical_dwell_s;

    // Hysteresis (voltage above entry threshold required to exit a level).
    float warn_hyst_V;
    float shed_hyst_V;
    float critical_hyst_V;

    // Maximum acceptable sample age; older samples are treated as invalid.
    float max_sample_age_s;
};

// Outputs emitted by the core. The shell maps these to FPP ports/events.
struct UvOutputs {
    bool emit_l1;   // request L1 shed: camera, gimbal, humidity sensor off
    bool emit_l2;   // request L2 shed: high-rate logger off, reduce tlm rate
    bool level_changed;
    UvLevel new_level;
    UvLevel old_level;
    bool sample_rejected;  // a sample was invalid or stale
};

class UvStateCore {
  public:
    UvStateCore();

    void configure(const UvConfig& cfg);
    void reset();

    // Inject a sample and the current time (seconds, monotonic).
    UvOutputs update(const PowerRailTlm& sample, float now_s);

    UvLevel level() const { return m_level; }
    float dwell_accum_s() const { return m_dwell_accum_s; }
    uint32_t rejected_count() const { return m_rejected_count; }
    uint32_t transition_count() const { return m_transition_count; }

  private:
    UvLevel classify(float voltage_V) const;
    bool may_exit(UvLevel current, float voltage_V) const;
    float dwell_required(UvLevel target) const;
    UvOutputs make_transition(UvLevel old_level, UvLevel new_level);

    UvConfig m_cfg;
    UvLevel m_level;
    float m_dwell_accum_s;
    float m_last_time_s;
    bool m_have_last_time;
    uint32_t m_rejected_count;
    uint32_t m_transition_count;
};

}  // namespace UvState
}  // namespace Components

#endif
