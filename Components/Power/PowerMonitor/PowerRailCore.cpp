#include "PowerRailCore.hpp"

namespace PowerRail {

PowerRailCore::PowerRailCore()
    : frozen_threshold_s_(1.0f) {
    // Default config: 3.3V ref, 12-bit ADC, unity divider, zero offset.
    cfg_.vref_V = 3.3f;
    cfg_.full_scale = 4095u;
    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        cfg_.divider_ratio[i] = 1.0f;
        cfg_.cal_offset_V[i] = 0.0f;

        samples_[i].rail_id = i;
        samples_[i].voltage_V = quietNaN();
        samples_[i].current_A = quietNaN();
        samples_[i].sample_age_s = 0.0f;
        samples_[i].valid = false;

        last_counts_[i] = 0;
        have_last_counts_[i] = false;
        last_change_time_[i] = 0.0;
        last_good_time_[i] = 0.0;
        have_good_[i] = false;
    }
}

void PowerRailCore::configure(const AdcConfig& cfg) {
    cfg_ = cfg;
}

float PowerRailCore::countsToVoltage(RailId rail, uint32_t counts) const {
    if (cfg_.full_scale == 0u) {
        return quietNaN();
    }
    float vadc = (static_cast<float>(counts) / static_cast<float>(cfg_.full_scale)) * cfg_.vref_V;
    float vrail = vadc * cfg_.divider_ratio[rail] + cfg_.cal_offset_V[rail];
    return vrail;
}

bool PowerRailCore::tick(double now_s, IAdcReader& adc) {
    bool any_good = false;

    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        RailId rail = static_cast<RailId>(i);
        RailSample& s = samples_[i];

        bool ok = false;
        uint32_t counts = adc.read(rail, ok);

        if (!ok) {
            // Read failure: invalid, age keeps accumulating.
            s.valid = false;
            if (have_good_[i]) {
                s.sample_age_s = static_cast<float>(now_s - last_good_time_[i]);
            } else {
                s.sample_age_s = 0.0f;
            }
            // Do not update last_counts_ on failure.
            continue;
        }

        // Frozen detection: counts unchanged for too long.
        bool frozen = false;
        if (have_last_counts_[i] && counts == last_counts_[i]) {
            double unchanged = now_s - last_change_time_[i];
            if (unchanged > static_cast<double>(frozen_threshold_s_)) {
                frozen = true;
            }
        } else {
            last_counts_[i] = counts;
            have_last_counts_[i] = true;
            last_change_time_[i] = now_s;
        }

        if (frozen) {
            s.valid = false;
            if (have_good_[i]) {
                s.sample_age_s = static_cast<float>(now_s - last_good_time_[i]);
            }
            continue;
        }

        // Good sample.
        s.rail_id = i;
        s.voltage_V = countsToVoltage(rail, counts);
        s.current_A = quietNaN(); // current not measured by this backend
        s.valid = true;
        s.sample_age_s = 0.0f;
        last_good_time_[i] = now_s;
        have_good_[i] = true;
        any_good = true;
    }

    return any_good;
}

const RailSample& PowerRailCore::sample(RailId rail) const {
    return samples_[rail];
}

} // namespace PowerRail
