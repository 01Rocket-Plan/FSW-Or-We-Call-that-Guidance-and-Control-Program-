#include "Components/Gnss/GnssDriver/GnssCore.hpp"
#include <cmath>
#include <cstdlib>
#include <cstring>
namespace Gnss {
namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kKnot2Mps = 0.514444;
constexpr double kKmh2Mps = 1.0 / 3.6;
constexpr int kGpsUtcLeap_s = 18;  // GPS-UTC 閏秒 (2017 起)
int hexv(char c) { return c >= '0' && c <= '9' ? c - '0' : (c >= 'A' && c <= 'F' ? c - 'A' + 10 : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1)); }
// 切欄位 (就地以 '\0' 取代 ',')
int split(char* s, char** f, int maxF) { int n = 0; f[n++] = s; for (char* p = s; *p && n < maxF; ++p) if (*p == ',') { *p = 0; f[n++] = p + 1; } return n; }
double nmeaAngle_rad(const char* v, const char* h) {  // ddmm.mmmm -> rad
  if (!*v) return NAN; double x = atof(v); int deg = int(x / 100); double a = deg + (x - deg * 100) / 60.0;
  if (*h == 'S' || *h == 'W') a = -a; return a * kDeg2Rad; }
long daysFromCivil(int y, unsigned m, unsigned d) { y -= m <= 2; long era = (y >= 0 ? y : y - 399) / 400; unsigned yoe = unsigned(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1; unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; return era * 146097 + long(doe) - 719468; }
uint32_t fnv(const char* s) { uint32_t h = 2166136261u; while (*s) { h ^= uint8_t(*s++); h *= 16777619u; } return h; }
}  // namespace

void GnssCore::start() {
  m_fix = GnssFix{}; m_fix.velEnu_mps[2] = NAN; m_fix.lat_rad = m_fix.lon_rad = NAN;
  m_fixLostLatched = false; m_noDataLatched = false; m_hadPosValid = false;
  m_noDataTicks = 0; m_sameGgaTicks = 0; m_gotValidSentence = false; m_haveFix = false;
  setState(DrvState::INIT);
}

void GnssCore::setState(DrvState s) { if (s == m_state) return; DrvState f = m_state; m_state = s; m_stateTicks = 0; m_sink.onStateChange(f, s); }

bool GnssCore::computePosOk(float dataAge_s) const {
  if (m_fix.fixType != FixType::FIX_3D) return false;
  if (!(m_fix.hdop <= m_p.hdopMax)) return false;
  if (dataAge_s > m_p.fixStaleAge_s) return false;
  return true;
}

void GnssCore::updateLockEvents(bool posOk) {
  // Gate C6：FIX_FAILED／失鎖只報一次；恢復後再報 FIX_RECOVERED，之後可再失鎖再報一次。
  if (posOk) {
    m_hadPosValid = true;
    if (m_fixLostLatched) {
      m_fixLostLatched = false;
      m_sink.onFault(GnssFault::FIX_RECOVERED, 1);
    }
  } else if (m_hadPosValid && !m_fixLostLatched) {
    m_fixLostLatched = true;
    m_sink.onFault(GnssFault::FIX_LOST, 1);
  }
}

void GnssCore::tick(double t_s) {
  if (m_state == DrvState::OFF) return;
  m_newGga = false;
  uint8_t buf[512]; int32_t n = m_uart.readBytes(buf, sizeof(buf));  // 每 tick 只讀一次
  if (n < 0) { m_sink.onFault(GnssFault::BUS_ERROR, 1); setState(DrvState::FAULT); return; }
  for (int32_t i = 0; i < n; ++i) feedByte(buf[i], t_s);
  if (n == 0) {
    ++m_noDataTicks;
  } else {
    m_noDataTicks = 0;
    m_noDataLatched = false;  // 位元組恢復 → 允許下一段靜默再報一次 NO_DATA
  }
  ++m_stateTicks;
  switch (m_state) {
    case DrvState::INIT: setState(DrvState::SELFTEST); break;  // UART 已由 F´ 開好，直接進自測
    case DrvState::SELFTEST:
      if (m_gotValidSentence) setState(DrvState::READY);
      else if (m_stateTicks >= m_p.selftestTicks) { m_sink.onFault(GnssFault::SELFTEST_FAIL, 1); setState(DrvState::FAULT); }
      break;
    case DrvState::READY: {
      // C6：飛行中靜默／失鎖不自己中止。預設 noDataFaultTicks=0 → 永不因靜默進 FAULT。
      if (m_p.noDataWarnTicks > 0 && m_noDataTicks >= m_p.noDataWarnTicks && !m_noDataLatched) {
        m_noDataLatched = true;
        m_sink.onFault(GnssFault::NO_DATA, m_noDataTicks);
      }
      if (m_p.noDataFaultTicks > 0 && m_noDataTicks >= m_p.noDataFaultTicks) {
        m_sink.onFault(GnssFault::NO_DATA, m_noDataTicks);
        setState(DrvState::FAULT);
        break;
      }
      // 假設 A2：STUCK 視為硬體異常 → FAULT
      if (m_sameGgaTicks >= m_p.stuckTicks) {
        m_sink.onFault(GnssFault::STUCK, m_sameGgaTicks);
        setState(DrvState::FAULT);
        break;
      }
      break;
    }
    case DrvState::FAULT:  // 恢復: 重新收到新鮮的合法 GGA 即回 SELFTEST
      if (m_newGga && m_sameGgaTicks == 0) { m_gotValidSentence = false; setState(DrvState::SELFTEST); }
      break;
    default: break;
  }
  if (m_haveFix && m_state == DrvState::READY) emitFix(t_s);
}

void GnssCore::emitFix(double t_s) {
  // READY 中每 tick 輸出一次 (seq 每次輸出 +1)；dataAge_s = 現在 - 最後一次收到新 GGA
  GnssFix f = m_fix; f.seq = ++m_seq; f.t_s = t_s; f.valid = true;
  f.dataAge_s = float(t_s - m_lastFixT_s);
  const bool posOk = computePosOk(f.dataAge_s);
  f.posValid = posOk;
  updateLockEvents(posOk);
  m_sink.onFix(f);
}

void GnssCore::feedByte(uint8_t b, double t_s) {
  if (b == '$') { if (m_inLine && m_len > 0) { ++m_trunc; m_sink.onFault(GnssFault::TRUNCATED, m_trunc); } m_inLine = true; m_len = 0; m_line[m_len++] = '$'; return; }
  if (!m_inLine) return;
  if (b == '\r' || b == '\n') { m_line[m_len] = 0; m_inLine = false; feedSentence(m_line, t_s); return; }
  if (m_len >= sizeof(m_line) - 1) { ++m_trunc; m_inLine = false; m_sink.onFault(GnssFault::TRUNCATED, m_trunc); return; }
  m_line[m_len++] = char(b);
}

bool GnssCore::feedSentence(const char* s, double t_s) {
  size_t L = strlen(s); const char* star = strrchr(s, '*');
  if (L < 7 || s[0] != '$' || !star || size_t(star - s) + 3 != L) { ++m_trunc; m_sink.onFault(GnssFault::TRUNCATED, m_trunc); return false; }
  uint8_t ck = 0; for (const char* p = s + 1; p < star; ++p) ck ^= uint8_t(*p);
  int h1 = hexv(star[1]), h2 = hexv(star[2]);
  if (h1 < 0 || h2 < 0 || uint8_t(h1 * 16 + h2) != ck) { ++m_badCk; m_sink.onFault(GnssFault::BAD_CHECKSUM, m_badCk); return false; }
  char body[96]; size_t bl = size_t(star - s - 1); memcpy(body, s + 1, bl); body[bl] = 0;
  m_gotValidSentence = true; handleSentence(body, t_s); return true;
}

void GnssCore::handleSentence(char* body, double t_s) {
  char* f[24]; int n = split(body, f, 24); const char* id = f[0];
  if (strlen(id) < 5) return; const char* type = id + 2;  // GP/GN/BD 前綴皆可
  if (!strcmp(type, "GGA") && n >= 10) {
    uint32_t h = fnv(f[1]) ^ fnv(f[2]) ^ fnv(f[4]);
    m_sameGgaTicks = (h == m_lastGgaHash) ? m_sameGgaTicks + 1 : 0; m_lastGgaHash = h;
    int q = atoi(f[6]);
    m_fix.numSats = uint8_t(atoi(f[7])); m_fix.hdop = *f[8] ? float(atof(f[8])) : 99.0f;
    if (q == 0) { m_fix.fixType = FixType::NO_FIX; m_fix.lat_rad = m_fix.lon_rad = NAN; m_fix.altMsl_m = NAN; }
    else { m_fix.lat_rad = nmeaAngle_rad(f[2], f[3]); m_fix.lon_rad = nmeaAngle_rad(f[4], f[5]);
           m_fix.altMsl_m = *f[9] ? float(atof(f[9])) : NAN;
           m_fix.fixType = (m_gsaMode == 2) ? FixType::FIX_2D : FixType::FIX_3D; if (std::isnan(m_fix.altMsl_m)) m_fix.fixType = FixType::FIX_2D; }
    if (m_sameGgaTicks == 0) { m_newGga = true; m_haveFix = true; m_lastFixT_s = t_s; }
  } else if (!strcmp(type, "GSA") && n >= 3) {
    m_gsaMode = uint8_t(atoi(f[2]));
    if (m_gsaMode == 2 && m_fix.fixType == FixType::FIX_3D) m_fix.fixType = FixType::FIX_2D;
    if (m_gsaMode <= 1) m_fix.fixType = FixType::NO_FIX;
  } else if (!strcmp(type, "RMC") && n >= 10 && strlen(f[1]) >= 6 && strlen(f[9]) == 6) {
    int hh = (f[1][0]-'0')*10+(f[1][1]-'0'), mm = (f[1][2]-'0')*10+(f[1][3]-'0'); double ss = atof(f[1] + 4);
    int dd = (f[9][0]-'0')*10+(f[9][1]-'0'), mo = (f[9][2]-'0')*10+(f[9][3]-'0'), yy = 2000+(f[9][4]-'0')*10+(f[9][5]-'0');
    double gps_s = double(daysFromCivil(yy, mo, dd) - daysFromCivil(1980, 1, 6)) * 86400.0 + hh * 3600 + mm * 60 + ss + kGpsUtcLeap_s;
    m_fix.gpsWeek = uint16_t(gps_s / 604800.0); m_fix.gpsTow_s = gps_s - m_fix.gpsWeek * 604800.0;
  } else if (!strcmp(type, "VTG") && n >= 8) {
    bool hasSpd = *f[7] || *f[5]; bool hasCrs = *f[1];
    if (hasSpd && hasCrs) { double v = *f[7] ? atof(f[7]) * kKmh2Mps : atof(f[5]) * kKnot2Mps; double c = atof(f[1]) * kDeg2Rad;
      m_fix.velEnu_mps[0] = float(v * sin(c)); m_fix.velEnu_mps[1] = float(v * cos(c)); m_fix.velHValid = true; }
    else { m_fix.velEnu_mps[0] = m_fix.velEnu_mps[1] = NAN; m_fix.velHValid = false; }
    m_fix.velEnu_mps[2] = NAN; m_fix.velUValid = false;  // NMEA 無垂直速度: U 一律 NaN
  }
}
}  // namespace Gnss
