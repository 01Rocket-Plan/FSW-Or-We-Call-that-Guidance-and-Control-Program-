// ======================================================================
// \title  TlmSend.cpp
// \brief  tlmSend F´ 外殼：run → TlmSendCore.tick → LogicalPacket → pktOut[層]。
// ======================================================================
#include "Components/Wireless/TlmSend/TlmSend.hpp"

namespace Components {

TlmSend::TlmSend(const char* compName)
    : TlmSendComponentBase(compName), m_core(), m_paramsLoaded(false), m_runCount(0), m_pktsSent(0), m_tlmDiv(0), m_pktBuf{} {
  m_core.setTimeSource(this);
  m_core.setSink(this);
  m_core.setProvider(this);
  m_core.setEventSink(this);
  m_core.setSource(TlmSource::DS);
  // 預設速率＝FPP 預設（P1 10、P2 10、P3 100、P4 200 拍 @ rg100Hz）；parametersLoaded 會再套 prmDb 值
  (void)m_core.addChannel(kApidP1, 10);
  (void)m_core.addChannel(kApidP2, 10);
  (void)m_core.addChannel(kApidP3, 100);
  (void)m_core.addChannel(kApidP4, 200);
}

// F´ 開機逐個呼叫 parameterUpdated（其他參數還沒載完）→ 等 parametersLoaded 一次套
void TlmSend::parameterUpdated(FwPrmIdType) {
  if (m_paramsLoaded) {
    loadParams();
  }
}
void TlmSend::parametersLoaded() {
  m_paramsLoaded = true;
  loadParams();
}

void TlmSend::loadParams() {
  Fw::ParamValid v;
  (void)m_core.setRateDiv(kApidP1, this->paramGet_APID_RATE_DIV_P1(v));
  (void)m_core.setRateDiv(kApidP2, this->paramGet_APID_RATE_DIV_P2(v));
  (void)m_core.setRateDiv(kApidP3, this->paramGet_APID_RATE_DIV_P3(v));
  (void)m_core.setRateDiv(kApidP4, this->paramGet_APID_RATE_DIV_P4(v));
}

void TlmSend::run_handler(FwIndexType, U32) {
  ++m_runCount;
  m_core.tick();
  if (++m_tlmDiv >= kTlmEveryTicks) {
    m_tlmDiv = 0;
    this->tlmWrite_PktsSent(m_pktsSent);
    this->tlmWrite_TimeNonMonoCount(m_core.timeNonMonoCount());
  }
}

void TlmSend::SET_ENABLED_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U16 apid, U8 enabled) {
  const bool ok = m_core.setEnabled(apid, enabled != 0);
  this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::VALIDATION_ERROR);
}

double TlmSend::now_s() {
  const Fw::Time now = this->getTime();
  return static_cast<double>(now.getSeconds()) + static_cast<double>(now.getUSeconds()) * 1e-6;
}

FwIndexType TlmSend::portForApid(U16 apid) {
  switch (apid) {
    case kApidP1:
      return 0;
    case kApidP2:
      return 1;
    case kApidP3:
      return 2;
    default:
      return 3;  // P4（含未分類）
  }
}

uint16_t TlmSend::sample(uint16_t apid, uint8_t* buf, uint16_t cap) {
  // TODO(無線組)：tlmIn 埠（各元件頻道，合約 §9）還沒有；接上前先送 4 B 佔位酬載＝tlmSend run 計數（大端），
  // 讓 SIL 的 tlmSend → comQueue → framer → rfDriver 下行鏈真的有幀在跑。不含任何飛行狀態或任務門檻。
  (void)apid;
  if (buf == nullptr || cap < 4) return 0;
  buf[0] = static_cast<uint8_t>(m_runCount >> 24);
  buf[1] = static_cast<uint8_t>(m_runCount >> 16);
  buf[2] = static_cast<uint8_t>(m_runCount >> 8);
  buf[3] = static_cast<uint8_t>(m_runCount);
  return 4;
}

void TlmSend::onPacket(const TlmPacket& p) {
  const FwIndexType port = portForApid(p.apid);
  if (!this->isConnected_pktOut_OutputPort(port)) return;
  WirelessPkt::Header h;
  h.type = static_cast<uint8_t>(p.type);
  h.source = static_cast<uint8_t>(p.src);
  h.apid = p.apid;
  h.seq = p.hdr.seq;
  h.bootCount = p.bootCount;
  h.t_s = p.hdr.t_s;
  h.valid = p.hdr.valid;
  const size_t n = WirelessPkt::encode(h, p.payload, p.payloadLen, m_pktBuf, sizeof(m_pktBuf));
  if (n == 0) return;
  Fw::Buffer b(m_pktBuf, static_cast<Fw::Buffer::SizeType>(n));
  ComCfg::FrameContext ctx;
  ctx.set_comQueueIndex(port + 1);  // 優先層號（P1..P4）
  this->pktOut_out(port, b, ctx);
  ++m_pktsSent;
}

void TlmSend::onTimeNonMonotonic(uint16_t apid, double prev_s, double cur_s) {
  this->log_WARNING_LO_TimeNonMonotonic(apid, prev_s, cur_s);
}

}  // namespace Components
