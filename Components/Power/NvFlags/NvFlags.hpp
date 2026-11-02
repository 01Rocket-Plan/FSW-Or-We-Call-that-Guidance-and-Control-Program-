#ifndef COMPONENTS_POWER_NVFLAGS_NVFLAGS_HPP
#define COMPONENTS_POWER_NVFLAGS_NVFLAGS_HPP

#include "NvFlagsCore.hpp"
#include "INvStore.hpp"
#include "Components/Power/NvFlags/NvFlagsComponentAc.hpp"

namespace Components {

//! NvFlags F´ shell. Base ID 0x6100 (Sim 0x6180).
class NvFlags final : public NvFlagsComponentBase {
  public:
    NvFlags(const char* const compName, ::NvFlags::INvStore& store);
    ~NvFlags() override;

    void configure();

  private:
    Fw::Success sepReleasedSet_handler(FwIndexType portNum, U8 mask);
    U8 sepReleasedGet_handler(FwIndexType portNum);
    Fw::Success sepCommanded_handler(FwIndexType portNum);
    Fw::Success dsChuteCommanded_handler(FwIndexType portNum);
    U32 phaseGet_handler(FwIndexType portNum);
    Fw::Success phaseSet_handler(FwIndexType portNum, U32 phase);

    void publishState();

    ::NvFlags::NvFlagsCore m_core;
};

}  // namespace Components

#endif
