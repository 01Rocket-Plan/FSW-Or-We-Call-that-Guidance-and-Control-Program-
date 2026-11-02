#include "PowerRailSim.hpp"

namespace PowerRail {

PowerRailSim::PowerRailSim() : now_s_(0.0) {
    cfg_.vref_V = 3.3f;
    cfg_.full_scale = 4095u;
    for (uint8_t i = 0; i < NUM_RAILS; ++i) {
        cfg_.divider_ratio[i] = 1.0f;
        cfg_.cal_offset_V[i] = 0.0f;

        state_[i].profile = CONSTANT;
        state_[i].base_V = 1.0f;
        state_[i].ramp_rate = 0.0f;
        state_[i].drop_time_s = 1e9;
        state_[i].drop_V = 0.0f;
        state_[i].read_fail = false;
        state_[i].frozen = false;
        state_[i].frozen_counts = 0;
    }
}

void PowerRailSim::setProfile(RailId rail, Profile p, float base_V) {
    state_[rail].profile = p;
    state_[rail].base_V = base_V;
}

void PowerRailSim::setRampRate(RailId rail, float volts_per_s) {
    state_[rail].ramp_rate = volts_per_s;
}

void PowerRailSim::setStepDrop(RailId rail, double drop_time_s, float drop_V) {
    state_[rail].drop_time_s = drop_time_s;
    state_[rail].drop_V = drop_V;
}

void PowerRailSim::setReadFail(RailId rail, bool fail) {
    state_[rail].read_fail = fail;
}

void PowerRailSim::setFrozen(RailId rail, bool frozen) {
    state_[rail].frozen = frozen;
    if (frozen) {
        state_[rail].frozen_counts = voltageToCounts(rail, state_[rail].base_V);
    }
}

void PowerRailSim::advance(double now_s) {
    now_s_ = now_s;
}

uint32_t PowerRailSim::voltageToCounts(RailId rail, float v) const {
    if (cfg_.full_scale == 0u || cfg_.divider_ratio[rail] == 0.0f) {
        return 0u;
    }
    float vadc = (v - cfg_.cal_offset_V[rail]) / cfg_.divider_ratio[rail];
    float frac = vadc / cfg_.vref_V;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    return static_cast<uint32_t>(frac * static_cast<float>(cfg_.full_scale) + 0.5f);
}

uint32_t PowerRailSim::read(RailId rail, bool& ok) {
    RailState& st = state_[rail];

    if (st.read_fail) {
        ok = false;
        return 0u;
    }

    if (st.frozen) {
        ok = true;
        return st.frozen_counts;
    }

    float v = st.base_V;
    switch (st.profile) {
        case CONSTANT:
            break;
        case RAMP:
            v = st.base_V + st.ramp_rate * static_cast<float>(now_s_);
            break;
        case STEP_DROP:
            if (now_s_ >= st.drop_time_s) {
                v = st.base_V - st.drop_V;
            }
            break;
    }

    ok = true;
    return voltageToCounts(rail, v);
}

} // namespace PowerRail
