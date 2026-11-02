// Gnss::GnssFix → RocketTypes::GnssFix（FPP 產生型別）。給 Edward／審閱者：欄位一對一對應 v0.2。
// 安裝位置：DS/US 皆可。僅在 F´ 元件編譯單元使用（需 ComponentAc／Serializable）。
#pragma once
#include "Components/Gnss/GnssDriver/GnssTypes.hpp"
#include "RocketTypes/GnssFixSerializableAc.hpp"
#include "RocketTypes/SampleHeaderSerializableAc.hpp"
#include "RocketTypes/FixTypeEnumAc.hpp"
#include <cstring>

namespace Gnss {

inline RocketTypes::GnssFix toFpp(const GnssFix& f) {
  RocketTypes::SampleHeader hdr(f.t_s, f.seq, f.valid);
  RocketTypes::FixType ft(
      static_cast<RocketTypes::FixType::T>(static_cast<U8>(f.fixType)));
  RocketTypes::GnssFix::Type_of_velEnu_mps vel = {
      f.velEnu_mps[0], f.velEnu_mps[1], f.velEnu_mps[2]};
  return RocketTypes::GnssFix(hdr, f.gpsTow_s, f.gpsWeek, ft, f.numSats, f.hdop,
                              f.lat_rad, f.lon_rad, f.altMsl_m, vel,
                              f.posValid, f.velHValid, f.velUValid, f.dataAge_s);
}

}  // namespace Gnss
