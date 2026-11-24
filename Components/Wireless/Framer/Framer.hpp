// ======================================================================
// \title  Framer.hpp
// \brief  framer F´ 外殼（passive）。組幀全在 FramerCore；本檔解 LogicalPacket、轉發 framedOut。
//
// comIn：comQueue 送來的 LogicalPacket（Components/Wireless/Ports/LogicalPacket.hpp）。
// framedOut：§2 空中幀 → rfDriver.dataIn。下游在呼叫內同步複製（全鏈被動），所以用元件內暫存，
// bufferAllocate／bufferDeallocate 目前不接（之後改非同步再接 bufferManager）。
// ======================================================================
#ifndef COMPONENTS_WIRELESS_FRAMER_HPP
#define COMPONENTS_WIRELESS_FRAMER_HPP

#include "Components/Wireless/Framer/FramerComponentAc.hpp"
#include "Components/Wireless/Framer/FramerCore.hpp"

namespace Components {

class Framer final : public FramerComponentBase, private IFramerEvents {
 public:
  explicit Framer(const char* compName);
  ~Framer() override = default;

  //! 幀暫存大小（SX1278 單包上限 255 B；frameMax_B 設更大時 Core 回 BUFFER_TOO_SMALL）
  static constexpr U32 kFrameBufBytes = 255;
  //! 自訂錯誤碼（FramerCore 的 FramerError 用 1..4）：輸入不是合法 LogicalPacket
  static constexpr U8 kErrBadLogicalPacket = 100;

 private:
  // ---- 埠
  void comIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;

  // ---- 參數
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;
  void loadParams();

  // ---- Core 事件
  void onFramerError(FramerError code, uint32_t detail) override;

  FramerCore m_core;
  bool m_paramsLoaded;
  U32 m_framesBuilt;
  U32 m_rejects;
  U32 m_lastLen;
  U8 m_frameBuf[kFrameBufBytes];
};

}  // namespace Components

#endif
