#include "../Core/UvStateCore.hpp"

#include <cstdio>
#include <cstdlib>

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("CHECK FAILED: %s (%s:%d)\n", #cond, __FILE__,       \
                        __LINE__);                                           \
            std::exit(1);                                                    \
        }                                                                    \
    } while (0)

using namespace Components::UvState;

static UvConfig makeConfig() {
    UvConfig c;
    c.warn_entry_V = 11.0f;
    c.shed_entry_V = 10.5f;
    c.critical_entry_V = 10.0f;
    c.warn_dwell_s = 1.0f;
    c.shed_dwell_s = 2.0f;
    c.critical_dwell_s = 3.0f;
    c.warn_hyst_V = 0.2f;
    c.shed_hyst_V = 0.2f;
    c.critical_hyst_V = 0.2f;
    c.max_sample_age_s = 5.0f;
    return c;
}

static PowerRailTlm makeSample(float v, bool valid = true, float age = 0.0f) {
    PowerRailTlm s;
    s.rail_id = 1;
    s.voltage_V = v;
    s.current_A = 1.0f;
    s.sample_age_s = age;
    s.valid = valid;
    return s;
}

static void test_enter_each_level() {
    UvStateCore core;
    core.configure(makeConfig());
    core.reset();

    float t = 0.0f;
    CHECK(core.level() == NOMINAL);

    // Enter WARN: voltage below 11.0, dwell 1.0s (dt accumulates across ticks)
    core.update(makeSample(10.8f), t);  // t=0, dt=0
    t = 0.5f;
    core.update(makeSample(10.8f), t);  // dt=0.5, dwell=0.5
    CHECK(core.level() == NOMINAL);
    t = 1.1f;
    core.update(makeSample(10.8f), t);  // dt=0.6, dwell=1.1 >= 1.0
    CHECK(core.level() == UV_WARN);

    // Enter SHED: voltage below 10.5, dwell 2.0s
    core.update(makeSample(10.3f), t);  // start SHED dwell
    t = 2.1f;
    core.update(makeSample(10.3f), t);  // dt=1.0, dwell=1.0
    CHECK(core.level() == UV_WARN);
    t = 3.3f;
    core.update(makeSample(10.3f), t);  // dt=1.2, dwell=2.2 >= 2.0
    CHECK(core.level() == UV_SHED);

    // Enter CRITICAL: voltage below 10.0, dwell 3.0s
    core.update(makeSample(9.8f), t);
    t = 4.8f;
    core.update(makeSample(9.8f), t);  // dt=1.5
    CHECK(core.level() == UV_SHED);
    t = 6.5f;
    core.update(makeSample(9.8f), t);  // dt=1.7, dwell=3.2 >= 3.0
    CHECK(core.level() == UV_CRITICAL);
}

static void test_exit_each_level() {
    UvStateCore core;
    core.configure(makeConfig());
    core.reset();

    float t = 0.0f;
    // Drive to CRITICAL (dwell 3s)
    core.update(makeSample(9.8f), t);
    t = 3.5f;
    core.update(makeSample(9.8f), t);
    CHECK(core.level() == UV_CRITICAL);

    // Exit CRITICAL requires > 10.0 + 0.2 = 10.2; immediate once met.
    t = 3.6f;
    core.update(makeSample(10.1f), t);
    CHECK(core.level() == UV_CRITICAL);  // not enough margin

    t = 3.7f;
    core.update(makeSample(10.3f), t);
    CHECK(core.level() == UV_SHED);

    // Exit SHED requires > 10.5 + 0.2 = 10.7
    t = 3.8f;
    core.update(makeSample(10.6f), t);
    CHECK(core.level() == UV_SHED);
    t = 3.9f;
    core.update(makeSample(10.8f), t);
    CHECK(core.level() == UV_WARN);

    // Exit WARN requires > 11.0 + 0.2 = 11.2
    t = 4.0f;
    core.update(makeSample(11.1f), t);
    CHECK(core.level() == UV_WARN);
    t = 4.1f;
    core.update(makeSample(11.3f), t);
    CHECK(core.level() == NOMINAL);
}

static void test_hysteresis_no_chatter() {
    UvStateCore core;
    core.configure(makeConfig());
    core.reset();

    float t = 0.0f;
    core.update(makeSample(10.8f), t);
    t = 1.1f;
    core.update(makeSample(10.8f), t);
    CHECK(core.level() == UV_WARN);

    // Oscillate around entry threshold (11.0) but below exit (11.2).
    for (int i = 0; i < 20; ++i) {
        float v = (i % 2 == 0) ? 10.95f : 11.05f;
        t += 0.1f;
        core.update(makeSample(v), t);
        CHECK(core.level() == UV_WARN);
    }
    CHECK(core.transition_count() == 1);
}

static void test_dwell_timing() {
    UvStateCore core;
    core.configure(makeConfig());
    core.reset();

    float t = 0.0f;
    core.update(makeSample(10.8f), t);
    t = 0.4f;
    core.update(makeSample(10.8f), t);
    CHECK(core.level() == NOMINAL);
    t = 0.8f;
    core.update(makeSample(10.8f), t);
    CHECK(core.level() == NOMINAL);
    t = 1.2f;  // total dwell dt = 0.4+0.4+0.4 = 1.2
    core.update(makeSample(10.8f), t);
    CHECK(core.level() == UV_WARN);

    // Now target SHED (dwell 2.0s)
    core.update(makeSample(10.3f), t);
    t = 2.2f;
    core.update(makeSample(10.3f), t);  // dt=1.0
    CHECK(core.level() == UV_WARN);
    t = 3.4f;
    core.update(makeSample(10.3f), t);  // dt=1.2, dwell=2.2
    CHECK(core.level() == UV_SHED);
}

static void test_invalid_samples_ignored_for_dwell() {
    UvStateCore core;
    core.configure(makeConfig());
    core.reset();

    float t = 0.0f;
    for (int i = 0; i < 10; ++i) {
        core.update(makeSample(10.8f, false), t);
        t += 0.5f;
    }
    CHECK(core.level() == NOMINAL);
    CHECK(core.rejected_count() == 10);

    // Stale sample (age > max) also rejected.
    core.update(makeSample(10.8f, true, 100.0f), t);
    t += 0.5f;
    CHECK(core.level() == NOMINAL);
    CHECK(core.rejected_count() == 11);

    // Valid samples now accumulate and transition.
    core.update(makeSample(10.8f), t);
    t += 0.6f;
    core.update(makeSample(10.8f), t);
    t += 0.6f;
    core.update(makeSample(10.8f), t);
    CHECK(core.level() == UV_WARN);
}

static void test_shed_outputs() {
    UvStateCore core;
    core.configure(makeConfig());
    core.reset();

    float t = 0.0f;
    // Drive to SHED (need shed dwell 2.0; 10.3 also below warn so warn
    // would need 1.0 first — classify jumps to SHED target with shed dwell)
    core.update(makeSample(10.3f), t);
    t = 2.5f;
    UvOutputs out = core.update(makeSample(10.3f), t);
    CHECK(core.level() == UV_SHED);
    CHECK(out.emit_l1 == true);
    CHECK(out.emit_l2 == false);

    // Drive to CRITICAL
    core.update(makeSample(9.8f), t);
    t = 6.0f;
    out = core.update(makeSample(9.8f), t);
    CHECK(core.level() == UV_CRITICAL);
    CHECK(out.emit_l1 == true);
    CHECK(out.emit_l2 == true);
}

int main() {
    test_enter_each_level();
    test_exit_each_level();
    test_hysteresis_no_chatter();
    test_dwell_timing();
    test_invalid_samples_ignored_for_dwell();
    test_shed_outputs();
    std::printf("All UvState host tests passed.\n");
    return 0;
}
