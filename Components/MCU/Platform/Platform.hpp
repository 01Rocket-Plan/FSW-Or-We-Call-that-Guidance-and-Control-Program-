// ======================================================================
// \title  Platform.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  F´ 外殼；邏輯在 PlatformCore。Base ID 0x1000。
// ======================================================================
#ifndef COMPONENTS_MCU_PLATFORM_PLATFORM_HPP
#define COMPONENTS_MCU_PLATFORM_PLATFORM_HPP

#include "Components/MCU/Platform/PlatformComponentAc.hpp"
#include "Components/MCU/Platform/PlatformCore.hpp"

namespace Mcu {

class Platform final : public PlatformComponentBase {
 public:
  explicit Platform(const char* compName);
  ~Platform() override;

  //! 拓樸設定階段呼叫；不呼叫就用內建的 LinuxPlatformHal（SIL）。
  void setHal(IPlatformHal& hal) { m_core.setHal(hal); }

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void CLEAR_MAX_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void loadParams();

  LinuxPlatformHal m_defaultHal;
  PlatformCore m_core;
  U32 m_tlmDiv;
  U32 m_overrunEvents;
  bool m_bootReported;
};

}  // namespace Mcu

#endif
