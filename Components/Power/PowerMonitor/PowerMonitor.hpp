#ifndef COMPONENTS_POWER_POWERMONITOR_HPP
#define COMPONENTS_POWER_POWERMONITOR_HPP

#include "PowerRailCore.hpp"
#include "PowerRailSim.hpp"

#include "Components/Power/PowerMonitor/PowerMonitorComponentAc.hpp"

namespace Components {

//! F´ shell for the voltage-rail sampler. Base ID 0x6000 (Sim 0x6080).
class PowerMonitor : public PowerMonitorComponentBase {
  public:
    PowerMonitor(const char* const compName);
    ~PowerMonitor();

  private:
    void schedIn_handler(FwIndexType portNum, U32 context) override;

    void SET_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, bool enabled) override;
    void SIM_READ_FAIL_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                  const PowerRailId& rail, bool fail) override;
    void SIM_FROZEN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                               const PowerRailId& rail, bool frozen) override;

    void parametersLoaded() override;
    void parameterUpdated(FwPrmIdType id) override;

    void applyParams();
    void emitTelemetry();

    ::PowerRail::PowerRailCore core_;
    ::PowerRail::PowerRailSim  sim_;
    bool use_sim_;
    bool prev_valid_[::PowerRail::NUM_RAILS];
};

} // namespace Components

#endif
