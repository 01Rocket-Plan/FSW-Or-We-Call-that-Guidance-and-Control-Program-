#include "Components/Gnss/GnssDriver/GnssCore.hpp"
#include <cmath>
#include <cstdlib>
#include <cstring>
namespace Gnss {
namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kKnot2Mps = 0.514444;
constexpr double kKmh2Mps = 1.0 / 3.6;
constexpr int kGpsUtcLeap_s = 18;
int hexv(char c) {
  return c >= '0' && c <= '9' ? c - '0'
       : (c >= 'A' && c <= 'F' ? c - 'A' + 10
       : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1));
}
int split(char* s, char** f, int maxF) {
  int n = 0; f[n++] = s;
  for (char* p = s; *p && n < maxF; ++p) if (*p == ',') { *p = 0; f[n++] = p + 1; }
  return n;
}
double nmeaAngle_rad(const char* v, const char* h) {
  if (!*v) return NAN;
  double x = atof(v); int deg = int(x / 100); double a = deg + (x - deg * 100) / 60.0;
  if (*h == 'S' || *h == 'W') a = -a;
  return a * kDeg2Rad;
}
long daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2; long era = (y >= 0 ? y : y - 399) / 400; unsigned yoe = unsigned(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + long(doe) - 719468;
}
uint32_t fnv(const char* s) {
  uint32_t h = 2166136261u; while (*s) { h ^= uint8_t(*s++); h *= 16777619u; } return h;
}
}  // namespace

void GnssCore::start() {
  m_fix = GnssFix{}; m_fix.velEnu_mps[2] = NAN; m_fix.lat_rad = m_fix.lon_rad = NAN;
  m_lock = LockStage::ACQ; m_lastPoll_s = -1e9; m_lastNewGga_s = -1e9; m_lastGoodPos_s = -1e9;
  m_lastEmit_s = -1e9; m_sameGgaPolls = 0; m_gotValidSentence = false; m_haveFix = false;
  m_fixFailedMsgSent = false; m_acqTimeoutSent = false; m_pollsInStage = 0;
  m_len = 0; m_inLine = false; m_seq = 0;
  setState(DrvState::INIT);
}

void GnssCore::setState(DrvState s) {
  if (s == m_state) return;
  DrvState f = m_state; m_state = s;
  m_sink.onStateChange(f, s);
}

void GnssCore::setLock(LockStage s, double t_s) {
  if (s == m_lock) return;
  LockStage f = m_lock; m_lock = s; m_stageEnter_s = t_s; m_pollsInStage = 0;
  if (s != LockStage::FIX_FAILED) m_fixFailedMsgSent = false;
  m_sink.onLockStageChange(f, s);
}

float GnssCore::pollInterval_s() const {
  switch (m_lock) {
    case LockStage::TRACKING:
    case LockStage::ACQ:         return m_p.pollTracking_s;
    case LockStage::REACQ_FAST:  return m_p.pollReacqFast_s;
    case LockStage::REACQ_MED:   return m_p.pollReacqMed_s;
    case LockStage::REACQ_SLOW:
    case LockStage::FIX_FAILED:  return m_p.pollReacqSlow_s;
  }
  return m_p.pollTracking_s;
}

bool GnssCore::qualityOk() const {
  return m_fix.fixType == FixType::FIX_3D && m_fix.hdop <= m_p.hdopMax;
}

double GnssCore::outageAge_s(double t_s) const {
  // 55 s 時鐘＝自最後合格定位；尚無合格點時退回階段進入時刻（開機 ACQ→FIX_FAILED）
  if (m_lastGoodPos_s > -1e8) return t_s - m_lastGoodPos_s;
  return t_s - m_stageEnter_s;
}

bool GnssCore::detectLoss(double t_s) const {
  // A3／A5：不以單次空 poll 當失鎖；看「無新 GGA」時長與 STUCK、以及本 poll 收到的劣質 GGA
  if (m_gotGgaThisPoll && !qualityOk()) return true;
  if (m_lastNewGga_s > -1e8 && (t_s - m_lastNewGga_s) > m_p.lossNoGga_s) return true;
  if (m_p.stuckPolls > 0 && m_sameGgaPolls >= m_p.stuckPolls) return true;
  return false;
}

void GnssCore::onGoodFix(double t_s) {
  m_lastGoodPos_s = t_s;
  if (m_lock != LockStage::TRACKING) setLock(LockStage::TRACKING, t_s);
}

void GnssCore::advanceReacq(double t_s) {
  // 邊界＝自 last-good 起 15／25／55（LOSS_NO_GGA 偵測耗時算進 FAST）
  const double fastEnd = m_p.reacqFastDur_s;
  const double medEnd  = m_p.reacqFastDur_s + m_p.reacqMedDur_s;
  const double slowEnd = medEnd + m_p.reacqSlowDur_s;
  const double age = outageAge_s(t_s);

  if (age >= slowEnd) {
    if (m_lock != LockStage::FIX_FAILED) {
      setLock(LockStage::FIX_FAILED, t_s);
      if (!m_fixFailedMsgSent) {
        m_fixFailedMsgSent = true;
        m_sink.onFault(GnssFault::FIX_FAILED, 1);
        m_sink.onFixFailedOpenSky();  // 「請檢察環境開放條件以利天線運作」
      }
    }
  } else if (age >= medEnd) {
    if (m_lock != LockStage::REACQ_SLOW && m_lock != LockStage::FIX_FAILED)
      setLock(LockStage::REACQ_SLOW, t_s);
  } else if (age >= fastEnd) {
    if (m_lock == LockStage::REACQ_FAST) setLock(LockStage::REACQ_MED, t_s);
  }
}

void GnssCore::tick(double t_s) {
  if (m_state == DrvState::OFF) return;

  if (m_state == DrvState::INIT) {
    setState(DrvState::SELFTEST);
    m_selftestEnter_s = t_s;
    m_lastPoll_s = t_s - pollInterval_s();  // 立刻允許第一次 poll
  }

  if (m_state == DrvState::READY) {
    // A1：開機 ACQ 逾時 → 一次事件後進 FIX_FAILED（不 FAULT）
    if (m_lock == LockStage::ACQ && (t_s - m_acqEnter_s) >= m_p.initAcqTimeout_s) {
      if (!m_acqTimeoutSent) {
        m_acqTimeoutSent = true;
        m_sink.onFault(GnssFault::INIT_ACQ_TIMEOUT, 1);
      }
      if (m_lock != LockStage::FIX_FAILED) {
        setLock(LockStage::FIX_FAILED, t_s);
        if (!m_fixFailedMsgSent) {
          m_fixFailedMsgSent = true;
          m_sink.onFault(GnssFault::FIX_FAILED, 1);
          m_sink.onFixFailedOpenSky();
        }
      }
    }
    if (m_lock == LockStage::REACQ_FAST || m_lock == LockStage::REACQ_MED || m_lock == LockStage::REACQ_SLOW)
      advanceReacq(t_s);
  }
  if (m_state == DrvState::SELFTEST && (t_s - m_selftestEnter_s) >= m_p.selftestTimeout_s) {
    if (!m_gotValidSentence) {
      m_sink.onFault(GnssFault::SELFTEST_FAIL, 1);
      setState(DrvState::FAULT);
      return;
    }
  }

  const float iv = (m_state == DrvState::SELFTEST || m_state == DrvState::FAULT)
                     ? m_p.pollTracking_s : pollInterval_s();
  if (t_s - m_lastPoll_s + 1e-12 < iv) return;
  m_lastPoll_s = t_s;
  doPoll(t_s);
}

void GnssCore::doPoll(double t_s) {
  m_newGga = false; m_gotGgaThisPoll = false;
  ++m_pollsInStage;
  uint8_t buf[kPollReadMax];
  int32_t n = m_uart.readBytes(buf, sizeof(buf));  // 每個 poll 最多一次
  if (n < 0) { m_sink.onFault(GnssFault::BUS_ERROR, 1); setState(DrvState::FAULT); return; }
  for (int32_t i = 0; i < n; ++i) feedByte(buf[i], t_s);

  if (m_state == DrvState::SELFTEST) {
    if (m_gotValidSentence) {
      setState(DrvState::READY);
      m_acqEnter_s = t_s;
      setLock(LockStage::ACQ, t_s);
      if (qualityOk()) onGoodFix(t_s);  // 首句已是合格 3D → 直接 TRACKING
      maybeEmit(t_s);
    }
    return;
  }
  if (m_state == DrvState::FAULT) {
    // BUS／SELFTEST_FAIL：鎖存至再次 start()
    return;
  }
  if (m_state == DrvState::READY) {
    if (qualityOk() && m_newGga) {
      onGoodFix(t_s);
    } else if (m_lock == LockStage::TRACKING && detectLoss(t_s)) {
      setLock(LockStage::REACQ_FAST, t_s);
    } else if (m_lock == LockStage::REACQ_FAST || m_lock == LockStage::REACQ_MED
               || m_lock == LockStage::REACQ_SLOW) {
      advanceReacq(t_s);
    }
    maybeEmit(t_s);
  }
}

void GnssCore::emitFix(double t_s, bool posValid) {
  GnssFix f = m_fix; f.seq = ++m_seq; f.t_s = t_s; f.valid = true;
  f.posValid = posValid;
  if (m_lastGoodPos_s > -1e8) f.dataAge_s = float(t_s - m_lastGoodPos_s);
  else if (m_lastNewGga_s > -1e8) f.dataAge_s = float(t_s - m_lastNewGga_s);
  else f.dataAge_s = 0.f;
  m_lastEmit_s = t_s;
  m_sink.onFix(f);
}

void GnssCore::maybeEmit(double t_s) {
  if (m_newGga) {
    const bool ok = (m_lock == LockStage::TRACKING) && qualityOk();
    emitFix(t_s, ok);
    return;
  }
  // 重鎖／靜默／FIX_FAILED／ACQ：無新 GGA 時最多 1 Hz 降級，讓 dataAge 進遙測
  if (!m_haveFix) return;
  if (m_lock == LockStage::TRACKING && qualityOk()) return;  // 鎖定間隙等下一句 GGA
  if (t_s - m_lastEmit_s + 1e-12 < 1.0) return;
  emitFix(t_s, false);
}

void GnssCore::feedByte(uint8_t b, double t_s) {
  if (b == '$') {
    if (m_inLine && m_len > 0) { ++m_trunc; m_sink.onFault(GnssFault::TRUNCATED, m_trunc); }
    m_inLine = true; m_len = 0; m_line[m_len++] = '$'; return;
  }
  if (!m_inLine) return;
  if (b == '\r' || b == '\n') { m_line[m_len] = 0; m_inLine = false; feedSentence(m_line, t_s); return; }
  if (m_len >= sizeof(m_line) - 1) { ++m_trunc; m_inLine = false; m_sink.onFault(GnssFault::TRUNCATED, m_trunc); return; }
  m_line[m_len++] = char(b);
}

bool GnssCore::feedSentence(const char* s, double t_s) {
  size_t L = strlen(s); const char* star = strrchr(s, '*');
  if (L < 7 || s[0] != '$' || !star || size_t(star - s) + 3 != L) {
    ++m_trunc; m_sink.onFault(GnssFault::TRUNCATED, m_trunc); return false;
  }
  uint8_t ck = 0; for (const char* p = s + 1; p < star; ++p) ck ^= uint8_t(*p);
  int h1 = hexv(star[1]), h2 = hexv(star[2]);
  if (h1 < 0 || h2 < 0 || uint8_t(h1 * 16 + h2) != ck) {
    ++m_badCk; m_sink.onFault(GnssFault::BAD_CHECKSUM, m_badCk); return false;
  }
  char body[96]; size_t bl = size_t(star - s - 1); memcpy(body, s + 1, bl); body[bl] = 0;
  m_gotValidSentence = true; handleSentence(body, t_s); return true;
}

void GnssCore::handleSentence(char* body, double t_s) {
  char* f[24]; int n = split(body, f, 24); const char* id = f[0];
  if (strlen(id) < 5) return; const char* type = id + 2;
  if (!strcmp(type, "GGA") && n >= 10) {
    uint32_t h = fnv(f[1]) ^ fnv(f[2]) ^ fnv(f[4]);
    m_sameGgaPolls = (h == m_lastGgaHash) ? m_sameGgaPolls + 1 : 0;
    m_lastGgaHash = h;
    int q = atoi(f[6]);
    m_fix.numSats = uint8_t(atoi(f[7]));
    m_fix.hdop = *f[8] ? float(atof(f[8])) : 99.0f;
    if (q == 0) {
      m_fix.fixType = FixType::NO_FIX; m_fix.lat_rad = m_fix.lon_rad = NAN; m_fix.altMsl_m = NAN;
    } else {
      m_fix.lat_rad = nmeaAngle_rad(f[2], f[3]); m_fix.lon_rad = nmeaAngle_rad(f[4], f[5]);
      m_fix.altMsl_m = *f[9] ? float(atof(f[9])) : NAN;
      m_fix.fixType = (m_gsaMode == 2) ? FixType::FIX_2D : FixType::FIX_3D;
      if (std::isnan(m_fix.altMsl_m)) m_fix.fixType = FixType::FIX_2D;
    }
    m_gotGgaThisPoll = true; m_haveFix = true; m_lastNewGga_s = t_s;
    if (m_sameGgaPolls == 0) m_newGga = true;
  } else if (!strcmp(type, "GSA") && n >= 3) {
    m_gsaMode = uint8_t(atoi(f[2]));
    if (m_gsaMode == 2 && m_fix.fixType == FixType::FIX_3D) m_fix.fixType = FixType::FIX_2D;
    if (m_gsaMode <= 1) m_fix.fixType = FixType::NO_FIX;
  } else if (!strcmp(type, "RMC") && n >= 10 && strlen(f[1]) >= 6 && strlen(f[9]) == 6) {
    int hh = (f[1][0]-'0')*10+(f[1][1]-'0'), mm = (f[1][2]-'0')*10+(f[1][3]-'0'); double ss = atof(f[1] + 4);
    int dd = (f[9][0]-'0')*10+(f[9][1]-'0'), mo = (f[9][2]-'0')*10+(f[9][3]-'0');
    int yy = 2000+(f[9][4]-'0')*10+(f[9][5]-'0');
    double gps_s = double(daysFromCivil(yy, mo, dd) - daysFromCivil(1980, 1, 6)) * 86400.0
                   + hh * 3600 + mm * 60 + ss + kGpsUtcLeap_s;
    m_fix.gpsWeek = uint16_t(gps_s / 604800.0); m_fix.gpsTow_s = gps_s - m_fix.gpsWeek * 604800.0;
  } else if (!strcmp(type, "VTG") && n >= 8) {
    bool hasSpd = *f[7] || *f[5]; bool hasCrs = *f[1];
    if (hasSpd && hasCrs) {
      double v = *f[7] ? atof(f[7]) * kKmh2Mps : atof(f[5]) * kKnot2Mps;
      double c = atof(f[1]) * kDeg2Rad;
      m_fix.velEnu_mps[0] = float(v * sin(c)); m_fix.velEnu_mps[1] = float(v * cos(c)); m_fix.velHValid = true;
    } else {
      m_fix.velEnu_mps[0] = m_fix.velEnu_mps[1] = NAN; m_fix.velHValid = false;
    }
    m_fix.velEnu_mps[2] = NAN; m_fix.velUValid = false;
  }
}
}  // namespace Gnss
