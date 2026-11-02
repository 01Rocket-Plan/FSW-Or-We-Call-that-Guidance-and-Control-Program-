// GnssTypes.hpp - GNSS 共用型別 (GnssFix v0.2, 首席工程師核定 2026-10-25)
// 安裝位置: DS/US 皆可 (依部署設定)。單位一律 SI，變數名帶單位後綴。
// Edward 2026-11-02 失鎖表：LockStage = TRACKING / REACQ_* / FIX_FAILED（另 ACQ＝開機搜星）。
#pragma once
#include <cstdint>
namespace Gnss {
enum class FixType : uint8_t { NO_FIX = 0, FIX_2D = 1, FIX_3D = 2 };
enum class DrvState : uint8_t { OFF = 0, INIT, SELFTEST, READY, FAULT };  // 握手；只走遙測
// Edward 2026-11-02 失鎖／重鎖階段（遙測 LockStage）
enum class LockStage : uint8_t {
  ACQ = 0,         // 開機初始搜星（最多 INIT_ACQ_TIMEOUT_S；逾時→FIX_FAILED）
  TRACKING = 1,    // 鎖定中，poll 100 ms
  REACQ_FAST = 2,  // 自 last-good →15 s；100 ms（含 LOSS_NO_GGA 偵測）
  REACQ_MED = 3,   // 15→25 s；1000 ms
  REACQ_SLOW = 4,  // 25→55 s；2000 ms
  FIX_FAILED = 5   // ≥55 s；原文一次；之後仍以 SLOW 輪詢
};
struct GnssFix {
  bool     valid;
  uint32_t seq;
  double   t_s;
  double   gpsTow_s;
  uint16_t gpsWeek;
  FixType  fixType;
  uint8_t  numSats;
  float    hdop;
  double   lat_rad;
  double   lon_rad;
  float    altMsl_m;
  float    velEnu_mps[3];
  bool     velHValid;
  bool     velUValid;
  bool     posValid;
  float    dataAge_s;
};
}  // namespace Gnss
