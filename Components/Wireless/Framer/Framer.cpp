// ======================================================================
// \title  Framer.cpp
// \brief  framer F´ 外殼：LogicalPacket → FramerCore.build → framedOut（rfDriver.dataIn）。
// ======================================================================
#include "Components/Wireless/Framer/Framer.hpp"

#include "Components/Wireless/Ports/LogicalPacket.hpp"

namespace Components {

Framer::Framer(const char* compName)
    : FramerComponentBase(compName), m_core(), m_paramsLoaded(false), m_framesBuilt(0), m_rejects(0), m_lastLen(0), m_frameBuf{} {
  m_core.setEventSink(this);
}

void Framer::parameterUpdated(FwPrmIdType) {
  if (m_paramsLoaded) {
    loadParams();
  }
}
void Framer::parametersLoaded() {
  m_paramsLoaded = true;
  loadParams();
}

void Framer::loadParams() {
  Fw::ParamValid v;
  m_core.setFrameMax(static_cast<size_t>(this->paramGet_frameMax_B(v)));
}

void Framer::comIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext& context) {
  WirelessPkt::Header h;
  const uint8_t* payload = nullptr;
  size_t payloadLen = 0;
  if (!WirelessPkt::decode(data.getData(), static_cast<size_t>(data.getSize()), h, payload, payloadLen)) {
    ++m_rejects;
    this->log_WARNING_LO_FramerError(kErrBadLogicalPacket, static_cast<U32>(data.getSize()));
    this->tlmWrite_RejectCount(m_rejects);
    return;
  }
  FrameRequest req;
  req.type = static_cast<FrameType>(h.type);
  req.source = static_cast<FrameSource>(h.source);
  req.apid = h.apid;
  req.seq = h.seq;
  req.bootCount = h.bootCount;
  req.t_s = h.t_s;
  req.valid = h.valid;
  req.payload = payload;
  req.payloadLen = static_cast<uint16_t>(payloadLen);
  const FrameResult r = m_core.build(req, m_frameBuf, sizeof(m_frameBuf));
  if (!r.ok) {
    ++m_rejects;  // Core 已發 onFramerError
    this->tlmWrite_RejectCount(m_rejects);
    return;
  }
  ++m_framesBuilt;
  m_lastLen = static_cast<U32>(r.len);
  this->tlmWrite_FramesBuilt(m_framesBuilt);
  this->tlmWrite_LastFrameLen(m_lastLen);
  if (this->isConnected_framedOut_OutputPort(0)) {
    Fw::Buffer b(m_frameBuf, static_cast<Fw::Buffer::SizeType>(r.len));
    this->framedOut_out(0, b, context);
  }
}

void Framer::onFramerError(FramerError code, uint32_t detail) {
  this->log_WARNING_LO_FramerError(static_cast<U8>(code), detail);
}

}  // namespace Components
