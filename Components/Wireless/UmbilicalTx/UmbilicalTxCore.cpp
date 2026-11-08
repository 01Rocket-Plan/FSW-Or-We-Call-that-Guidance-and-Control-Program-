#include "UmbilicalTxCore.hpp"

namespace PadLink {

bool UmbilicalTxCore::computeReady(double now_s) const {
  if (!m_haveBit) return false;
  if (now_s - m_bitAt_s > m_p.bitMaxAge_s) return false;
  if ((m_bit & kRocketBitHardMask) != kRocketBitHardMask) return false;
  return m_mode == kModePadIdle || m_mode == kModeArmed;
}

bool UmbilicalTxCore::tick(double now_s) {
  if (m_haveTx && (now_s - m_lastTx_s) < m_p.period_s) return false;
  m_haveTx = true;
  m_lastTx_s = now_s;

  RocketStatusMsg m;
  m.flightMode = m_mode;
  m.bitFlags = m_bit;
  m.ready = computeReady(now_s);
  const double mv = m_battV * 1000.0;
  m.battV_mV = (mv > 0.0) ? (mv > 65535.0 ? 65535u : static_cast<std::uint16_t>(mv + 0.5)) : 0u;
  m.bootCount = m_bootCount;

  std::uint8_t payload[kRocketStatusBytes];
  std::uint8_t frame[kMaxFrameBytes];
  const std::size_t plen = packRocketStatus(m, payload, sizeof(payload));
  const std::size_t flen =
      encodeFrame(PadMsgType::ROCKET_STATUS, m_seq++, payload, plen, frame, sizeof(frame));
  const int w = m_uart.write(frame, flen);
  if (w == static_cast<int>(flen)) {
    ++m_sent;
  } else {
    ++m_writeFail;  // 起飛後臍帶拔脫是預期的：只計數，不報錯
  }
  return true;
}

}  // namespace PadLink
