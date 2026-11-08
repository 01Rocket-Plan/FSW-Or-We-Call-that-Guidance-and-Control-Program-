#include "PadCmdGate.hpp"

#include <cstring>

namespace PadLink {

std::size_t PadCmdGate::buildMacMessage(std::uint8_t bootId, std::uint32_t counter,
                                        PadCmdOp op, std::uint8_t* out, std::size_t cap) {
  if (out == nullptr || cap < kMacMsgBytes) return 0;
  std::size_t off = 0;
  std::memcpy(out, kDomainTag, kDomainTagLen);
  off += kDomainTagLen;
  out[off++] = bootId;
  out[off++] = static_cast<std::uint8_t>(counter >> 24);
  out[off++] = static_cast<std::uint8_t>((counter >> 16) & 0xFFu);
  out[off++] = static_cast<std::uint8_t>((counter >> 8) & 0xFFu);
  out[off++] = static_cast<std::uint8_t>(counter & 0xFFu);
  out[off++] = static_cast<std::uint8_t>(op);
  return off;
}

PadCmdResult PadCmdGate::check(const GroundCmdMsg& cmd) {
  if (!requiresAuth(cmd.op)) {
    // ABORT／DISARM：往安全方向，不驗簽名、不動計數器【待 Edward 定】。
    return PadCmdResult::ACCEPTED;
  }
  if (m_auth == nullptr) return PadCmdResult::NO_VERIFIER;
  if (cmd.bootId != m_bootId) return PadCmdResult::WRONG_BOOT;
  if (cmd.counter == 0u || cmd.counter <= m_lastCounter) return PadCmdResult::REPLAY;
  std::uint8_t msg[kMacMsgBytes];
  const std::size_t n = buildMacMessage(cmd.bootId, cmd.counter, cmd.op, msg, sizeof(msg));
  if (n == 0) return PadCmdResult::BAD_FRAME;
  if (!m_auth->verify(msg, n, cmd.tag)) return PadCmdResult::AUTH_FAIL;
  m_lastCounter = cmd.counter;  // 只有驗過簽名才推進
  return PadCmdResult::ACCEPTED;
}

}  // namespace PadLink
