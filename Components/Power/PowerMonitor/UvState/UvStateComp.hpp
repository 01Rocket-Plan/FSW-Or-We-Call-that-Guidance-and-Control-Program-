#ifndef COMPONENTS_POWER_POWERMONITOR_UVSTATE_UVSTATECOMP_HPP
#define COMPONENTS_POWER_POWERMONITOR_UVSTATE_UVSTATECOMP_HPP

#include "Components/Power/PowerMonitor/UvState/UvStateCompComponentAc.hpp"
#include "Components/Power/PowerMonitor/UvState/Core/UvStateCore.hpp"

namespace Components {

//! UvState F´ shell. Base ID 0x6200 (within power range; Sim 0x6280).
class UvStateComp : public UvStateCompComponentBase {
  public:
    UvStateComp(const char* const compName);
    ~UvStateComp();

  private:
    void powerIn_handler(FwIndexType portNum, const PowerRailTlm& sample);
    void schedIn_handler(FwIndexType portNum, U32 context);

    void RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq);
    void RELOAD_CONFIG_cmdHandler(FwOpcodeType opCode, U32 cmdSeq);

    void loadConfigFromParams();
    void publishTelemetry();
    void handleOutputs(const ::Components::UvState::UvOutputs& out, U32 rail_id);

    ::Components::UvState::UvStateCore m_core;
};

}  // namespace Components

#endif
