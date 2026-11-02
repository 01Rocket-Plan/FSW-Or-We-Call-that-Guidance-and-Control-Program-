// Host unit tests for PowerRailCore and PowerRailSim.
// Plain C++ with CHECK macros and main(); no F´ or gtest dependency.

#include "../PowerRailCore.hpp"
#include "../PowerRailSim.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

static int g_failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        ++g_failures; \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps) do { \
    double _a = (a); double _b = (b); \
    if (std::fabs(_a - _b) > (eps)) { \
        std::printf("FAIL %s:%d: |%f - %f| > %f\n", __FILE__, __LINE__, _a, _b, (double)(eps)); \
        ++g_failures; \
    } \
} while (0)

using namespace PowerRail;

static void test_adc_to_voltage() {
    PowerRailCore core;
    AdcConfig cfg;
    cfg.vref_V = 3.3f;
    cfg.full_scale = 4095u;
    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        cfg.divider_ratio[i] = 1.0f;
        cfg.cal_offset_V[i] = 0.0f;
    }
    cfg.divider_ratio[BATT_MAIN] = 5.0f; // 5:1 divider
    core.configure(cfg);

    // Full scale -> 3.3V ADC -> 16.5V rail.
    CHECK_NEAR(core.countsToVoltage(BATT_MAIN, 4095u), 16.5, 1e-3);
    // Half scale -> 1.65V ADC -> 8.25V rail.
    CHECK_NEAR(core.countsToVoltage(BATT_MAIN, 2048u), 8.25, 1e-2);
    // Zero counts -> 0V.
    CHECK_NEAR(core.countsToVoltage(BATT_MAIN, 0u), 0.0, 1e-6);
}

static void test_nan_current() {
    PowerRailCore core;
    PowerRailSim sim;
    AdcConfig cfg;
    cfg.vref_V = 3.3f;
    cfg.full_scale = 4095u;
    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        cfg.divider_ratio[i] = 1.0f;
        cfg.cal_offset_V[i] = 0.0f;
    }
    core.configure(cfg);
    sim.setAdcConfig(cfg);
    sim.setProfile(LOGIC_3V3, PowerRailSim::CONSTANT, 3.3f);

    sim.advance(0.0);
    core.tick(0.0, sim);

    const RailSample& s = core.sample(LOGIC_3V3);
    CHECK(s.valid);
    CHECK(std::isnan(s.current_A));
}

static void test_read_fail_accumulates_age() {
    PowerRailCore core;
    PowerRailSim sim;
    AdcConfig cfg;
    cfg.vref_V = 3.3f;
    cfg.full_scale = 4095u;
    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        cfg.divider_ratio[i] = 1.0f;
        cfg.cal_offset_V[i] = 0.0f;
    }
    core.configure(cfg);
    sim.setAdcConfig(cfg);
    sim.setProfile(SERVO_5V, PowerRailSim::CONSTANT, 5.0f);

    // Good sample at t=0.
    sim.advance(0.0);
    core.tick(0.0, sim);
    CHECK(core.sample(SERVO_5V).valid);

    // Fail reads from t=0.01 onward.
    sim.setReadFail(SERVO_5V, true);
    for (int k = 1; k <= 5; ++k) {
        double t = 0.01 * k;
        sim.advance(t);
        core.tick(t, sim);
        const RailSample& s = core.sample(SERVO_5V);
        CHECK(!s.valid);
        CHECK_NEAR(s.sample_age_s, t, 1e-4);
    }
}

static void test_frozen_detection() {
    PowerRailCore core;
    PowerRailSim sim;
    AdcConfig cfg;
    cfg.vref_V = 3.3f;
    cfg.full_scale = 4095u;
    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        cfg.divider_ratio[i] = 1.0f;
        cfg.cal_offset_V[i] = 0.0f;
    }
    core.configure(cfg);
    core.setFrozenThreshold(0.05f); // 50 ms
    sim.setAdcConfig(cfg);
    sim.setProfile(BATT_MAIN, PowerRailSim::CONSTANT, 12.0f);

    // First good sample.
    sim.advance(0.0);
    core.tick(0.0, sim);
    CHECK(core.sample(BATT_MAIN).valid);

    // Freeze counts; after threshold, should be invalid.
    sim.setFrozen(BATT_MAIN, true);
    for (int k = 1; k <= 10; ++k) {
        double t = 0.01 * k;
        sim.advance(t);
        core.tick(t, sim);
    }
    CHECK(!core.sample(BATT_MAIN).valid);
}

static void test_100hz_scheduling() {
    PowerRailCore core;
    PowerRailSim sim;
    AdcConfig cfg;
    cfg.vref_V = 3.3f;
    cfg.full_scale = 4095u;
    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        cfg.divider_ratio[i] = 1.0f;
        cfg.cal_offset_V[i] = 0.0f;
    }
    core.configure(cfg);
    sim.setAdcConfig(cfg);
    // Stay within ADC range (unity divider, vref 3.3 V): 2.0 + 1.0*1s = 3.0 V.
    sim.setProfile(LOGIC_3V3, PowerRailSim::RAMP, 2.0f);
    sim.setRampRate(LOGIC_3V3, 1.0f); // 1 V/s

    // 100 ticks at 0.01 s = 1.0 s.
    for (int k = 0; k <= 100; ++k) {
        double t = 0.01 * k;
        sim.advance(t);
        core.tick(t, sim);
    }
    const RailSample& s = core.sample(LOGIC_3V3);
    CHECK(s.valid);
    // After 1.0 s ramp at 1 V/s from 2.0 V -> 3.0 V (within 3.3 V full-scale).
    CHECK_NEAR(s.voltage_V, 3.0, 1e-2);
    CHECK_NEAR(s.sample_age_s, 0.0, 1e-6);
}

int main() {
    test_adc_to_voltage();
    test_nan_current();
    test_read_fail_accumulates_age();
    test_frozen_detection();
    test_100hz_scheduling();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d TEST(S) FAILED\n", g_failures);
    return 1;
}
