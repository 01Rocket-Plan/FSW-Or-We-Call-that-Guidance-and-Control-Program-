// PadCmdGate：地面 → 發射架指令驗證（Issue #24）
//
// 規則（對齊 gate C14／C15／H4）：
//  - 全部指令（ARM、LAUNCH、HEARTBEAT、ABORT、DISARM）一律要簽名，deny-by-default。
//    ABORT 要簽名（Edward 2026-11-10，#42 C1）：中止是任務終止的標誌，不能讓任何人不經授權就送。
//    簽名擋不住「自己人按錯」，那要靠地面站的二次確認；簽名擋的是「別人偽造」。
//    地面鏈路斷線時架子本來就會自己中止（groundLinkTimeout_s → GROUND_LINK_LOST），所以 ABORT
//    要簽名不會讓「想中止卻中止不了」。
//    地面鏈路是否「活著」只看簽過名的幀，偽造的心跳不能讓倒數在地面斷線後繼續走。
//    沒有接上驗證器（IPadCmdAuth == nullptr）→ 一律拒絕，架子永遠停在 SAFE。
//  - DISARM 也要簽名（2026-11-12 無線組審 PR #52）：handleCommand 在 ARMED「和 COUNTDOWN」都接受
//    DISARM 並退回 SAFE，所以免簽的 DISARM 在倒數中效果等同免簽的 ABORT，會讓 #42 C1 的決定被繞過。
//    也對齊無線組合約 v0.1b「全部指令預設 HMAC，只有雲台例外」。Edward 若書面決定 DISARM 免簽，
//    要同時把 DISARM 限定在 ARMED（不能停倒數），再改這裡。
//  - 防重放：簽名指令的計數器必須嚴格遞增；只放 RAM，每次開機歸 0。
//    開機時架子產生的 bootId 會在 PadStatus 廣播，地面簽名時必須帶入，
//    所以上一次開機錄下來的舊封包，在新的 bootId 下驗不過。
//  - 密碼／實體鑰匙：簽名只解「軟體鎖」。實體鑰匙在硬體上串聯點火迴路，
//    沒轉鑰匙電流物理上到不了引線（C15 第三輪、H4）。
//
// 金鑰不在本 repo；IPadCmdAuth 實作由 Edward 在地面設定載入（CONTRIBUTING 第 6 條）。
#ifndef WIRELESS_PADCMDGATE_HPP
#define WIRELESS_PADCMDGATE_HPP

#include "PadLinkCodec.hpp"

#include <cstddef>
#include <cstdint>

namespace PadLink {

//! 簽名驗證介面。預計接 UplinkAuth（PR #13）的 HmacSha256，取前 16 B。
class IPadCmdAuth {
 public:
  virtual ~IPadCmdAuth() = default;
  //! msg＝buildMacMessage() 的輸出；tag 為 kCmdTagBytes。必須用常數時間比較。
  virtual bool verify(const std::uint8_t* msg, std::size_t msgLen,
                      const std::uint8_t* tag) = 0;
};

enum class PadCmdResult : std::uint8_t {
  NONE = 0,
  ACCEPTED = 1,
  AUTH_FAIL = 2,     // 簽名不對
  NO_VERIFIER = 3,   // 沒接驗證器（deny-by-default）
  REPLAY = 4,        // 計數器沒有遞增
  WRONG_BOOT = 5,    // bootId 對不上（舊封包）
  BAD_FRAME = 6,     // 格式錯
  REJECTED = 7,      // 驗證通過，但目前狀態不允許（由 PadControllerCore 回填）
};

class PadCmdGate {
 public:
  static constexpr const char* kDomainTag = "01RP-PAD-v1";
  static constexpr std::size_t kDomainTagLen = 11;
  static constexpr std::size_t kMacMsgBytes = kDomainTagLen + 1 + 4 + 1;

  PadCmdGate(IPadCmdAuth* auth, std::uint8_t bootId) : m_auth(auth), m_bootId(bootId) {}

  //! 是否需要簽名：全部指令都要（含 ABORT、DISARM；不認得的 op 也要）。
  static bool requiresAuth(PadCmdOp op) {
    (void)op;
    return true;
  }

  //! MAC 輸入：domainTag || bootId || counter_be32 || opcode。回傳長度。
  static std::size_t buildMacMessage(std::uint8_t bootId, std::uint32_t counter, PadCmdOp op,
                                     std::uint8_t* out, std::size_t cap);

  PadCmdResult check(const GroundCmdMsg& cmd);

  std::uint8_t bootId() const { return m_bootId; }
  std::uint32_t lastCounter() const { return m_lastCounter; }

 private:
  IPadCmdAuth* m_auth;
  std::uint8_t m_bootId;
  std::uint32_t m_lastCounter = 0;
};

}  // namespace PadLink

#endif
