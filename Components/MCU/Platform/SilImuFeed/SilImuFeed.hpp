// ======================================================================
// \title  SilImuFeed.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  SIL 佔位 IMU。Base ID 0x1080。
// ======================================================================
#ifndef COMPONENTS_MCU_PLATFORM_SILIMUFEED_HPP
#define COMPONENTS_MCU_PLATFORM_SILIMUFEED_HPP

#include "Components/MCU/Platform/SilImuFeed/SilImuFeedComponentAc.hpp"

namespace Mcu {

class SilImuFeed final : public SilImuFeedComponentBase {
 public:
  explicit SilImuFeed(const char* compName);
  ~SilImuFeed() override;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;

  U32 m_seq;
  bool m_haveT0;
  F64 m_t0_s;
};

}  // namespace Mcu

#endif
