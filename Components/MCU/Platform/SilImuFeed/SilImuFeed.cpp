// ======================================================================
// \title  SilImuFeed.cpp
// \author MCU 組（EdwardWang+Claude）
// ======================================================================
#include "Components/MCU/Platform/SilImuFeed/SilImuFeed.hpp"

#include <cmath>

namespace Mcu {

SilImuFeed::SilImuFeed(const char* compName) : SilImuFeedComponentBase(compName), m_seq(0), m_haveT0(false), m_t0_s(0) {}

SilImuFeed::~SilImuFeed() {}

void SilImuFeed::schedIn_handler(FwIndexType, U32) {
  const Fw::Time now = this->getTime();
  const F64 t_s = static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
  if (!m_haveT0) {
    m_haveT0 = true;
    m_t0_s = t_s;
  }
  Fw::ParamValid v;
  const F32 g = this->paramGet_g_mps2(v);
  const F32 tilt = this->paramGet_railTilt_rad(v);
  const bool valid = (t_s - m_t0_s) >= static_cast<F64>(this->paramGet_readyDelay_s(v));

  const RocketTypes::SampleHeader hdr(t_s, ++m_seq, valid);
  const RocketTypes::ImuSample::Type_of_acc_mps2 acc = {g * std::cos(tilt), 0.0f, g * std::sin(tilt)};  // 機體 X 指鼻（conventions §2）
  const RocketTypes::ImuSample::Type_of_gyro_rads gyro = {0.0f, 0.0f, 0.0f};
  if (this->isConnected_imuOut_OutputPort(0)) this->imuOut_out(0, RocketTypes::ImuSample(hdr, acc, gyro, false));
  if ((m_seq % 1000u) == 0u) this->tlmWrite_seq(m_seq);
}

}  // namespace Mcu
