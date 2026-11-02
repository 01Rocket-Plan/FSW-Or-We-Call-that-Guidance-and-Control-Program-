#include "UvStateComp.hpp"

namespace Components {

UvStateComp::UvStateComp(const char* const compName)
    : UvStateCompComponentBase(compName) {}

UvStateComp::~UvStateComp() {}

void UvStateComp::loadConfigFromParams() {
    Fw::ParamValid valid;
    ::Components::UvState::UvConfig cfg;
    cfg.warn_entry_V = this->paramGet_warnEntry_V(valid);
    cfg.shed_entry_V = this->paramGet_shedEntry_V(valid);
    cfg.critical_entry_V = this->paramGet_criticalEntry_V(valid);
    cfg.warn_dwell_s = this->paramGet_warnDwell_s(valid);
    cfg.shed_dwell_s = this->paramGet_shedDwell_s(valid);
    cfg.critical_dwell_s = this->paramGet_criticalDwell_s(valid);
    cfg.warn_hyst_V = this->paramGet_warnHyst_V(valid);
    cfg.shed_hyst_V = this->paramGet_shedHyst_V(valid);
    cfg.critical_hyst_V = this->paramGet_criticalHyst_V(valid);
    cfg.max_sample_age_s = this->paramGet_maxSampleAge_s(valid);
    m_core.configure(cfg);
}

void UvStateComp::publishTelemetry() {
    this->tlmWrite_level(static_cast<RocketTypes::UvState>(m_core.level()));
    this->tlmWrite_dwellAccum_s(m_core.dwell_accum_s());
    this->tlmWrite_rejectedCount(m_core.rejected_count());
    this->tlmWrite_transitionCount(m_core.transition_count());
}

void UvStateComp::handleOutputs(const ::Components::UvState::UvOutputs& out,
                                U32 rail_id) {
    if (out.sample_rejected) {
        this->log_WARNING_LO_sampleRejected(rail_id);
    }
    if (out.level_changed) {
        auto oldL = static_cast<RocketTypes::UvState>(out.old_level);
        auto newL = static_cast<RocketTypes::UvState>(out.new_level);
        this->log_ACTIVITY_HI_levelChanged(oldL, newL);
        this->uvStateOut_out(0, newL);
    }
    if (out.emit_l1) {
        this->log_WARNING_HI_shedL1Requested(
            static_cast<RocketTypes::UvState>(out.new_level));
        this->shedL1_out(0);
    }
    if (out.emit_l2) {
        this->log_WARNING_HI_shedL2Requested(
            static_cast<RocketTypes::UvState>(out.new_level));
        this->shedL2_out(0);
    }
    publishTelemetry();
}

void UvStateComp::powerIn_handler(FwIndexType portNum, const PowerRailTlm& sample) {
    (void)portNum;
    Fw::Time t = this->getTime();
    float now_s = static_cast<float>(t.getSeconds()) +
                  static_cast<float>(t.getUSeconds()) * 1e-6f;

    ::Components::UvState::PowerRailTlm s;
    s.rail_id = static_cast<uint32_t>(sample.get_rail_id());
    s.voltage_V = sample.get_voltage_V();
    s.current_A = sample.get_current_A();
    s.sample_age_s = sample.get_sample_age_s();
    s.valid = sample.get_valid();

    auto out = m_core.update(s, now_s);
    handleOutputs(out, s.rail_id);
}

void UvStateComp::schedIn_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;
    publishTelemetry();
}

void UvStateComp::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_core.reset();
    publishTelemetry();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void UvStateComp::RELOAD_CONFIG_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    loadConfigFromParams();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Components
