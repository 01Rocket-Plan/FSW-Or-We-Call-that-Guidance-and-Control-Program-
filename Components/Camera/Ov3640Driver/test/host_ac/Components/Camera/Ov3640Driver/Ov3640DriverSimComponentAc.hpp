#ifndef OV3640_HOST_OV3640DRIVERSIMCOMPONENTAC_HPP
#define OV3640_HOST_OV3640DRIVERSIMCOMPONENTAC_HPP
#include "Components/Camera/Ov3640Driver/Ov3640DriverComponentAc.hpp"
// Sim base shares the same host surface for commands/tlm/image; extra SIM_* no-ops in adapter.
namespace Components {
using Ov3640DriverSimComponentBase = Ov3640DriverComponentBase;
}
#endif
