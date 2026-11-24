// ======================================================================
// \title  TlmSend.hpp
// \brief  tlmSend F´ 外殼（rg100Hz／Core 0）。排程、seq、t_s 全在 TlmSendCore；本檔只搬資料。
//
// pktOut[k] ＝ 優先層 P(k+1)（P1..P4，見 ComQueue README 分層表）；拓樸把 pktOut[k] 接 comQueue.comIn[k+1]。
// 輸出是 LogicalPacket（Components/Wireless/Ports/LogicalPacket.hpp），framer 解出標頭再組空中幀。
// ======================================================================
#ifndef COMPONENTS_WIRELESS_TLMSEND_HPP
#define COMPONENTS_WIRELESS_TLMSEND_HPP

#include "Components/Wireless/Ports/LogicalPacket.hpp"
#include "Components/Wireless/TlmSend/TlmSendComponentAc.hpp"
#include "Components/Wireless/TlmSend/TlmSendCore.hpp"

namespace Components {

class TlmSend final : public TlmSendComponentBase,
                      private ITlmTimeSource,
                      private ITlmSink,
                      private ITlmChannelProvider,
                      private ITlmSendEvents {
 public:
  explicit TlmSend(const char* compName);
  ~TlmSend() override = default;

  //! 合約 §3.1 各層 APID（P1 飛行／P2 健康／P3 診斷／P4 可犧牲）
  static constexpr U16 kApidP1 = 0x010;
  static constexpr U16 kApidP2 = 0x020;
  static constexpr U16 kApidP3 = 0x030;
  static constexpr U16 kApidP4 = 0x040;
  //! 遙測寫出間隔（rg100Hz 下 100 拍＝1 Hz）
  static constexpr U32 kTlmEveryTicks = 100;

 private:
  // ---- 埠／指令
  void run_handler(FwIndexType portNum, U32 context) override;
  void SET_ENABLED_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U16 apid, U8 enabled) override;

  // ---- 參數
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;
  void loadParams();

  // ---- Core 介面
  double now_s() override;
  void onPacket(const TlmPacket& p) override;
  uint16_t sample(uint16_t apid, uint8_t* buf, uint16_t cap) override;
  void onTimeNonMonotonic(uint16_t apid, double prev_s, double cur_s) override;

  static FwIndexType portForApid(U16 apid);

  TlmSendCore m_core;
  bool m_paramsLoaded;
  U32 m_runCount;
  U32 m_pktsSent;
  U32 m_tlmDiv;
  //! 下游 comQueue.comIn 在呼叫內同步複製（全鏈被動）→ 單一暫存即可
  U8 m_pktBuf[WirelessPkt::kMaxBytes];
};

}  // namespace Components

#endif
