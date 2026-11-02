// GnssDriverSim：同一個 GnssCore 接 GnssSimDevice（base id 0x5080 = 0x5000+0x80）
// 安裝位置：依部署設定（DS/US 皆可）。
#ifndef GNSS_GNSSDRIVERSIM_HPP
#define GNSS_GNSSDRIVERSIM_HPP
#include "Components/Gnss/GnssDriverSim/GnssDriverSimComponentAc.hpp"
#include "Components/Gnss/GnssDriver/GnssCore.hpp"  // 組長負責；API 假設見 PR 說明
#include "Components/Gnss/GnssDriverSim/GnssSimDevice.hpp"

namespace Components {

class GnssDriverSim final : public GnssDriverSimComponentBase, private Gnss::IGnssSink {
 public:
  explicit GnssDriverSim(const char* compName);
  ~GnssDriverSim() override = default;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void SIM_SET_TRAJ_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F64 lat0_deg, F64 lon0_deg, F32 alt0_m,
                               F32 velE_mps, F32 velN_mps, F32 velU_mps) override;
  void SIM_SET_NOISE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 posH_sigma_m, F32 posV_sigma_m, F32 vel_sigma_mps) override;
  void SIM_SET_LATENCY_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 latency_s, F32 dropout_prob) override;
  void SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Components::GnssSimFault& fault) override;
  void SIM_SET_HDOP_BIAS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 bias) override;

  void onFix(const Gnss::GnssFix& fix) override;
  void onStateChange(Gnss::DrvState from, Gnss::DrvState to) override;
  void onFault(Gnss::GnssFault fault, uint32_t count) override;

  Gnss::GnssSimDevice m_dev;
  Gnss::GnssCore m_core;
  F64 m_lastT_s;     // 上一個 tick 的 F´ 時間，用來算模擬器的 dt
  bool m_haveLastT;
  U32 m_faultCount;  // 累計故障次數（遙測）
  bool m_started;    // 已呼叫 core.start()
  Gnss::GnssCoreParams readParams();
};

}  // namespace Components
#endif
