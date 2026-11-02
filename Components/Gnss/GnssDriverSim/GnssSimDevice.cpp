// GnssSimDevice 實作：真值軌跡 → 加雜訊的量測 → NMEA 句子 → 依 baud 速率與延遲吐出位元組
#include "Components/Gnss/GnssDriverSim/GnssSimDevice.hpp"
#include <cmath>
#include <cstdio>

namespace Gnss {

static const double kEarthR_m = 6378137.0;  // WGS84 長半軸（小範圍平面近似）
static const double kPi = 3.14159265358979323846;
static const double kMps2Kn = 1.0 / 0.514444;

GnssSimDevice::GnssSimDevice() {}

uint32_t GnssSimDevice::rnd() {  // xorshift32：可重現的亂數，測試結果固定
  m_rng ^= m_rng << 13; m_rng ^= m_rng >> 17; m_rng ^= m_rng << 5; return m_rng;
}
double GnssSimDevice::gauss() {  // Box-Muller 標準常態
  double u1 = (rnd() + 1.0) / 4294967297.0, u2 = (rnd() + 1.0) / 4294967297.0;
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

void GnssSimDevice::setTrajectory(double lat0_rad, double lon0_rad, double alt0_m, const float v[3]) {
  m_lat0 = lat0_rad; m_lon0 = lon0_rad; m_alt0 = alt0_m;
  for (int i = 0; i < 3; i++) m_vel[i] = v[i];
}
void GnssSimDevice::setNoise(float h, float v, float vel) { m_sigPosH = h; m_sigPosV = v; m_sigVel = vel; }

void GnssSimDevice::setFault(GnssSimFault f) {
  m_fault = f;
  if (f == GnssSimFault::NO_DATA) { m_pending.clear(); m_wire.clear(); m_tx.clear(); }
}

void GnssSimDevice::truthAt(double t, double& lat, double& lon, double& alt) const {
  lat = m_lat0 + (m_vel[1] * t) / kEarthR_m;
  lon = m_lon0 + (m_vel[0] * t) / (kEarthR_m * std::cos(m_lat0));
  alt = m_alt0 + m_vel[2] * t;
}

uint8_t GnssSimDevice::nmeaChecksum(const std::string& body) {
  uint8_t c = 0; for (char ch : body) c ^= static_cast<uint8_t>(ch); return c;
}

static std::string sentence(const std::string& body, bool badSum) {
  uint8_t c = GnssSimDevice::nmeaChecksum(body);
  if (badSum) c ^= 0x5A;
  char tail[8]; std::snprintf(tail, sizeof tail, "*%02X\r\n", c);
  return "$" + body + tail;
}

static std::string fmtLat(double rad, char* hemi) {
  double d = std::fabs(rad * 180.0 / kPi); *hemi = rad >= 0 ? 'N' : 'S';
  int deg = static_cast<int>(d); char b[24]; std::snprintf(b, sizeof b, "%02d%08.5f", deg, (d - deg) * 60.0); return b;
}
static std::string fmtLon(double rad, char* hemi) {
  double d = std::fabs(rad * 180.0 / kPi); *hemi = rad >= 0 ? 'E' : 'W';
  int deg = static_cast<int>(d); char b[24]; std::snprintf(b, sizeof b, "%03d%08.5f", deg, (d - deg) * 60.0); return b;
}

void GnssSimDevice::makeEpoch(double tEp) {
  m_epochs++;
  // HDOP／衛星數隨機漫步
  m_hdop += static_cast<float>(0.1 * gauss()); if (m_hdop < 0.6f) m_hdop = 0.6f; if (m_hdop > 4.0f) m_hdop = 4.0f;
  float hdopOut = m_hdop + m_hdopBias; if (hdopOut < 0.1f) hdopOut = 0.1f; if (hdopOut > 99.f) hdopOut = 99.f;
  if ((rnd() & 7u) == 0) m_sats += (rnd() & 1u) ? 1 : -1; if (m_sats < 4) m_sats = 4; if (m_sats > 16) m_sats = 16;

  bool dropout = m_dropoutProb > 0.f && (rnd() / 4294967296.0) < m_dropoutProb;
  bool noFix = dropout || m_fault == GnssSimFault::NO_FIX;
  int mode = noFix ? 1 : (m_fault == GnssSimFault::FIX_2D_ONLY ? 2 : 3);

  // 量測 = 真值 + 雜訊（真值不離開本函式）
  double lat, lon, alt; truthAt(tEp, lat, lon, alt);
  lat += m_sigPosH * gauss() / kEarthR_m;
  lon += m_sigPosH * gauss() / (kEarthR_m * std::cos(m_lat0));
  alt += m_sigPosV * gauss();
  double ve = m_vel[0] + m_sigVel * gauss(), vn = m_vel[1] + m_sigVel * gauss();
  double sog_kn = std::sqrt(ve * ve + vn * vn) * kMps2Kn;
  double cog = std::atan2(ve, vn) * 180.0 / kPi; if (cog < 0) cog += 360.0;

  int sod = static_cast<int>(tEp) % 86400;  // 模擬 UTC 從 00:00:00 起算
  char hms[16]; std::snprintf(hms, sizeof hms, "%02d%02d%05.2f", sod / 3600, (sod / 60) % 60, std::fmod(tEp, 60.0));
  char nh, eh; std::string sLat = fmtLat(lat, &nh), sLon = fmtLon(lon, &eh);
  char b[160];
  std::string out;

  // GGA
  if (noFix) std::snprintf(b, sizeof b, "GPGGA,%s,,,,,0,%02d,99.99,,M,,M,,", hms, m_sats < 3 ? m_sats : 3);
  else std::snprintf(b, sizeof b, "GPGGA,%s,%s,%c,%s,%c,1,%02d,%.2f,%.1f,M,0.0,M,,", hms, sLat.c_str(), nh, sLon.c_str(), eh,
                     m_sats, hdopOut, mode == 3 ? alt : 0.0);
  std::string gga = sentence(b, m_fault == GnssSimFault::BAD_CHECKSUM);
  if (m_fault == GnssSimFault::TRUNCATED) gga = gga.substr(0, gga.size() / 2);  // 中途截斷，沒有 *hh\r\n
  out += gga;
  // RMC
  if (noFix) std::snprintf(b, sizeof b, "GPRMC,%s,V,,,,,,,010126,,,N", hms);
  else std::snprintf(b, sizeof b, "GPRMC,%s,A,%s,%c,%s,%c,%.2f,%.1f,010126,,,A", hms, sLat.c_str(), nh, sLon.c_str(), eh, sog_kn, cog);
  out += sentence(b, false);
  // VTG
  if (noFix) std::snprintf(b, sizeof b, "GPVTG,,T,,M,,N,,K,N");
  else std::snprintf(b, sizeof b, "GPVTG,%.1f,T,,M,%.2f,N,%.2f,K,A", cog, sog_kn, sog_kn * 1.852);
  out += sentence(b, false);
  // GSA（mode 1=無 fix、2=2D、3=3D）
  std::snprintf(b, sizeof b, "GPGSA,A,%d,,,,,,,,,,,,,%.2f,%.2f,%.2f", mode, hdopOut * 1.4, noFix ? 99.99 : hdopOut, hdopOut * 1.2);
  out += sentence(b, false);

  if (m_fault == GnssSimFault::STUCK) {
    if (m_lastEpochBytes.empty()) m_lastEpochBytes = out;
    out = m_lastEpochBytes;  // 一直重送同一組句子
  } else {
    m_lastEpochBytes = out;
  }
  m_pending.push_back(Pending{tEp + m_latency_s, out});
}

void GnssSimDevice::advance(double dt_s) {
  m_t_s += dt_s;
  while (m_nextEpoch_s <= m_t_s + 1e-9) {
    if (m_fault != GnssSimFault::NO_DATA) makeEpoch(m_nextEpoch_s);
    m_nextEpoch_s += 1.0;
  }
  // 釋出的 epoch 先進入 wire；再依 baud 灌入 RX（m_tx）。如此：
  // - FakeGnssCore 每 10 ms 讀一次時，每 tick ≤ baud/10*dt（T9c）
  // - GnssCore 每 100 ms poll 一次時，期間 advance 已把額度灌進 m_tx，一次 read 拿完整句
  while (!m_pending.empty() && m_pending.front().release_s <= m_t_s + 1e-9) {
    for (char c : m_pending.front().bytes) m_wire.push_back(static_cast<uint8_t>(c));
    m_pending.pop_front();
  }
  // 本 tick 最多灌 baud/10*dt 位元組進 RX；無 wire 時清零額度，避免下一個 burst 疊加
  m_byteCredit = m_baud / 10.0 * dt_s;
  while (m_byteCredit >= 1.0 && !m_wire.empty()) {
    m_tx.push_back(m_wire.front()); m_wire.pop_front(); m_byteCredit -= 1.0;
  }
}

int32_t GnssSimDevice::readBytes(uint8_t* buf, uint32_t max) {
  if (m_fault == GnssSimFault::NO_DATA) return 0;
  uint32_t n = 0;
  while (n < max && !m_tx.empty()) {
    buf[n++] = m_tx.front(); m_tx.pop_front();
  }
  return static_cast<int32_t>(n);
}

}  // namespace Gnss
