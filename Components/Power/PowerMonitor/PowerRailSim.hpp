#ifndef POWERRAIL_SIM_HPP
#define POWERRAIL_SIM_HPP

#include "PowerRailCore.hpp"

namespace PowerRail {

// Simulated ADC backend with configurable voltage profiles and fault injection.
class PowerRailSim : public IAdcReader {
  public:
    enum Profile {
        CONSTANT = 0,
        RAMP     = 1,
        STEP_DROP = 2
    };

    PowerRailSim();

    // Configure a rail's profile. base_V is the nominal rail voltage.
    void setProfile(RailId rail, Profile p, float base_V);
    void setRampRate(RailId rail, float volts_per_s);
    void setStepDrop(RailId rail, double drop_time_s, float drop_V);

    // Fault injection.
    void setReadFail(RailId rail, bool fail);
    void setFrozen(RailId rail, bool frozen);

    // Advance simulated time (called by the component each tick).
    void advance(double now_s);

    // IAdcReader
    uint32_t read(RailId rail, bool& ok) override;

    // Convert a rail voltage back to raw counts (for test setup).
    uint32_t voltageToCounts(RailId rail, float v) const;

    void setAdcConfig(const AdcConfig& cfg) { cfg_ = cfg; }

  private:
    struct RailState {
        Profile profile;
        float base_V;
        float ramp_rate;
        double drop_time_s;
        float drop_V;
        bool read_fail;
        bool frozen;
        uint32_t frozen_counts;
    };

    AdcConfig cfg_;
    RailState state_[NUM_RAILS];
    double now_s_;
};

} // namespace PowerRail

#endif
