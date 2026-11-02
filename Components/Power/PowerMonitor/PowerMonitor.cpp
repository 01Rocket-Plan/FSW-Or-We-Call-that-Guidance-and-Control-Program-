#include "PowerMonitor.hpp"

namespace Components {

PowerMonitor::PowerMonitor(const char* const compName)
    : PowerMonitorComponentBase(compName),
      use_sim_(true) {
    for (uint8_t i = 0; i < ::PowerRail::NUM_RAILS; ++i) {
        prev_valid_[i] = false;
    }
}

PowerMonitor::~PowerMonitor() {}

void PowerMonitor::applyParams() {
    Fw::ParamValid valid;
    ::PowerRail::AdcConfig cfg;
    cfg.vref_V = this->paramGet_VREF_V(valid);
    cfg.full_scale = this->paramGet_FULL_SCALE(valid);
    cfg.divider_ratio[::PowerRail::BATT_MAIN] = this->paramGet_DIVIDER_BATT_MAIN(valid);
    cfg.divider_ratio[::PowerRail::SERVO_5V] = this->paramGet_DIVIDER_SERVO_5V(valid);
    cfg.divider_ratio[::PowerRail::LOGIC_3V3] = this->paramGet_DIVIDER_LOGIC_3V3(valid);
    cfg.cal_offset_V[::PowerRail::BATT_MAIN] = this->paramGet_CAL_OFFSET_BATT_MAIN(valid);
    cfg.cal_offset_V[::PowerRail::SERVO_5V] = this->paramGet_CAL_OFFSET_SERVO_5V(valid);
    cfg.cal_offset_V[::PowerRail::LOGIC_3V3] = this->paramGet_CAL_OFFSET_LOGIC_3V3(valid);

    core_.configure(cfg);
    sim_.setAdcConfig(cfg);
    core_.setFrozenThreshold(this->paramGet_FROZEN_THRESHOLD_S(valid));
}

void PowerMonitor::parametersLoaded() {
    applyParams();
}

void PowerMonitor::parameterUpdated(FwPrmIdType id) {
    (void)id;
    applyParams();
}

void PowerMonitor::schedIn_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    Fw::Time t = this->getTime();
    double now_s = static_cast<double>(t.getSeconds()) +
                   static_cast<double>(t.getUSeconds()) * 1e-6;

    sim_.advance(now_s);
    (void)use_sim_;  // real ADC backend TBD; Sim is default
    ::PowerRail::IAdcReader& adc = static_cast<::PowerRail::IAdcReader&>(sim_);
    core_.tick(now_s, adc);

    for (uint8_t i = 0; i < ::PowerRail::NUM_RAILS; ++i) {
        const ::PowerRail::RailSample& s =
            core_.sample(static_cast<::PowerRail::RailId>(i));
        if (prev_valid_[i] && !s.valid) {
            this->log_WARNING_HI_RailReadFail(static_cast<PowerRailId>(i));
        }
        prev_valid_[i] = s.valid;
    }

    emitTelemetry();
}

void PowerMonitor::emitTelemetry() {
    for (uint8_t i = 0; i < ::PowerRail::NUM_RAILS; ++i) {
        const ::PowerRail::RailSample& s =
            core_.sample(static_cast<::PowerRail::RailId>(i));
        PowerRailTlm tlm;
        tlm.set_rail_id(static_cast<PowerRailId>(s.rail_id));
        tlm.set_voltage_V(s.voltage_V);
        tlm.set_current_A(s.current_A);
        tlm.set_sample_age_s(s.sample_age_s);
        tlm.set_valid(s.valid);
        this->tlmWrite_PowerRailTlm(tlm);
        this->railOut_out(0, tlm);
    }
}

void PowerMonitor::SET_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, bool enabled) {
    use_sim_ = enabled;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void PowerMonitor::SIM_READ_FAIL_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                            PowerRailId rail, bool fail) {
    sim_.setReadFail(static_cast<::PowerRail::RailId>(static_cast<U8>(rail)), fail);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void PowerMonitor::SIM_FROZEN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                         PowerRailId rail, bool frozen) {
    sim_.setFrozen(static_cast<::PowerRail::RailId>(static_cast<U8>(rail)), frozen);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

} // namespace Components
