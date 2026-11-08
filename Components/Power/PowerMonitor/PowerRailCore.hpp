#ifndef POWERRAIL_CORE_HPP
#define POWERRAIL_CORE_HPP

#include <limits>
#include <cstdint>
#include <cmath>

namespace PowerRail {

// Number of rails managed by this component.
static constexpr uint8_t NUM_RAILS = 3;

// Rail identifiers. Mirrored in FPP as PowerRailId enum.
enum RailId : uint8_t {
    BATT_MAIN  = 0,
    SERVO_5V   = 1,
    LOGIC_3V3  = 2
};

// Quiet NaN used when a measurement is not available.
inline float quietNaN() {
    return std::numeric_limits<float>::quiet_NaN();
}

// Hardware abstraction for an ADC reader.
// read() returns raw counts and sets ok=false on failure.
class IAdcReader {
  public:
    virtual ~IAdcReader() = default;
    virtual uint32_t read(RailId rail, bool& ok) = 0;
};

// Static configuration for ADC -> voltage conversion.
struct AdcConfig {
    float vref_V;          // ADC reference voltage
    uint32_t full_scale;   // counts at vref (e.g. 4095 for 12-bit)
    float divider_ratio[NUM_RAILS]; // Vrail = Vadc * divider_ratio
    float cal_offset_V[NUM_RAILS];  // additive calibration offset
};

// One telemetry sample for a rail.
struct RailSample {
    uint8_t rail_id;
    float   voltage_V;
    float   current_A;     // quiet NaN if unmeasured
    float   sample_age_s;  // seconds since last good sample
    bool    valid;
};

// Pure, host-testable core. No F´ includes, no dynamic allocation.
class PowerRailCore {
  public:
    PowerRailCore();

    void configure(const AdcConfig& cfg);

    // Called once per rate-group tick with injected time (seconds).
    // Returns true if a new good sample was produced this tick.
    bool tick(double now_s, IAdcReader& adc);

    // Access the latest sample for a rail.
    const RailSample& sample(RailId rail) const;

    // Frozen detection: if the raw counts are unchanged for longer than
    // frozen_threshold_s, mark the rail invalid.
    void setFrozenThreshold(float seconds) { frozen_threshold_s_ = seconds; }

    // Convert raw counts to rail voltage using config.
    float countsToVoltage(RailId rail, uint32_t counts) const;

  private:
    AdcConfig cfg_;
    RailSample samples_[NUM_RAILS];

    // Frozen detection state.
    uint32_t last_counts_[NUM_RAILS];
    bool     have_last_counts_[NUM_RAILS];
    double   last_change_time_[NUM_RAILS];
    float    frozen_threshold_s_;

    // Time of last good sample per rail (for age accumulation).
    double   last_good_time_[NUM_RAILS];
    bool     have_good_[NUM_RAILS];
};

} // namespace PowerRail

#endif
