// ======================================================================
// \title  PadController.hpp
// \brief  發射架主控 F´ 外殼（PadMcu）。邏輯全部在 PadControllerCore；本檔只搬資料。
//
// 簽名驗證器：目前「沒有」接（IPadCmdAuth＝nullptr）→ PadCmdGate deny-by-default，
// 所有地面指令回 NO_VERIFIER，架子停在 SAFE、不可能點火。飛行金鑰不在 repo（docs/key-handling.md）；
// 之後接 UplinkAuth 的 HMAC（金鑰從本機 secrets／NVS 載入）時，連 bootId 一起從拓樸傳進來。
// ======================================================================
#ifndef COMPONENTS_WIRELESS_PADCONTROLLER_HPP
#define COMPONENTS_WIRELESS_PADCONTROLLER_HPP

#include "Components/Wireless/PadController/PadControllerComponentAc.hpp"
#include "Components/Wireless/PadController/PadControllerCore.hpp"

namespace Components {

class PadController final : public PadControllerComponentBase, private PadLink::IPadEvents {
 public:
  explicit PadController(const char* compName);
  ~PadController() override = default;

  //! 每幾拍送一次狀態幀（radioOut／umbilicalOut）與遙測。1 kHz 下 100＝10 Hz。
  static constexpr U32 kStatusEveryTicks = 100;

 private:
  // ---- 埠
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void umbilicalIn_handler(FwIndexType portNum, Fw::Buffer& fwBuffer) override;
  void radioIn_handler(FwIndexType portNum, Fw::Buffer& fwBuffer) override;

  // ---- 參數
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;
  void loadParams();

  // ---- Core 事件
  void onStateChange(PadLink::PadState from, PadLink::PadState to, PadLink::PadReason reason) override;
  void onCommand(PadLink::PadCmdOp op, PadLink::PadCmdResult result, PadLink::PadReason reason) override;
  void onBadParams(const PadLink::PadParams& p) override;

  F64 nowSeconds();
  void sendFrames();
  void writeTlm();

  PadLink::PadControllerCore m_core;
  U32 m_tickDiv;
  bool m_paramsLoaded;
  //! 狀態幀暫存：下游 BufferSend 必須在呼叫內同步複製（本元件被動）
  U8 m_radioBuf[PadLink::kMaxFrameBytes];
  U8 m_umbBuf[PadLink::kMaxFrameBytes];
};

}  // namespace Components

#endif
