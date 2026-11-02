// GnssTypes.hpp - GNSS 共用型別 (GnssFix v0.2, 首席工程師核定 2026-10-25)
// 安裝位置: DS/US 皆可 (依部署設定)。單位一律 SI，變數名帶單位後綴。
#pragma once
#include <cstdint>
namespace Gnss {
enum class FixType : uint8_t { NO_FIX = 0, FIX_2D = 1, FIX_3D = 2 };
enum class DrvState : uint8_t { OFF = 0, INIT, SELFTEST, READY, FAULT };  // 只走遙測，不進 GnssFix
struct GnssFix {
  bool     valid;          // 共用標頭: 僅 READY 為 true
  uint32_t seq;            // 只在實際輸出時 +1
  double   t_s;            // F´ 時間服務
  double   gpsTow_s;       // 僅記錄用
  uint16_t gpsWeek;        // 僅記錄用
  FixType  fixType;
  uint8_t  numSats;
  float    hdop;
  double   lat_rad;        // WGS84
  double   lon_rad;        // WGS84
  float    altMsl_m;
  float    velEnu_mps[3];  // U 無垂直速度時為 NaN，絕不填 0
  bool     velHValid;
  bool     velUValid;
  bool     posValid;       // 3D fix 且 hdop <= prmDb 門檻
  float    dataAge_s;      // F´ 時間服務計算
};
}  // namespace Gnss
