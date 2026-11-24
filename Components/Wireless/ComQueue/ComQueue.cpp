// ======================================================================
// \title  ComQueue.cpp
// \brief  comQueue F´ 外殼：comIn[層] → Core.enqueue；run → 最多 maxPktsPerRun 幀 → buffOut（framer）。
// ======================================================================
#include "Components/Wireless/ComQueue/ComQueue.hpp"

#include <cstring>

#include "Components/Wireless/Ports/LogicalPacket.hpp"

namespace Components {

ComQueue::ComQueue(const char* compName)
    : ComQueueComponentBase(compName), m_core(), m_coreLock(), m_linkBusy(true), m_paramsLoaded(false), m_tlmDiv(0), m_txFrame() {
  m_core.setEventSink(this);
}

// F´ 開機逐個呼叫 parameterUpdated（其他參數還沒載完）→ 等 parametersLoaded 一次套
void ComQueue::parameterUpdated(FwPrmIdType) {
  if (m_paramsLoaded) {
    loadParams();
  }
}
void ComQueue::parametersLoaded() {
  m_paramsLoaded = true;
  loadParams();
}

void ComQueue::loadParams() {
  // 注意：Core::setDepth 會清空該層（改深度只在地面做，合約 §3.2「飛行中不改」）
  Fw::ParamValid v;
  Os::ScopeLock lock(m_coreLock);
  (void)m_core.setDepth(ComPriority::P0, this->paramGet_depthP0(v));
  (void)m_core.setDepth(ComPriority::P1, this->paramGet_depthP1(v));
  (void)m_core.setDepth(ComPriority::P2, this->paramGet_depthP2(v));
  (void)m_core.setDepth(ComPriority::P3, this->paramGet_depthP3(v));
  (void)m_core.setDepth(ComPriority::P4, this->paramGet_depthP4(v));
  (void)m_core.setMaxPktsPerRun(this->paramGet_maxPktsPerRun(v));
}

void ComQueue::comIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext&) {
  if (portNum < 0 || portNum >= static_cast<FwIndexType>(ComPriority::NUM)) return;
  const U8* bytes = data.getData();
  const size_t len = static_cast<size_t>(data.getSize());
  if (bytes == nullptr || len < WirelessPkt::kHeaderBytes || len > sizeof(m_txFrame.data)) {
    return;  // 不是 LogicalPacket：丟棄（上游被動同步呼叫，不回收 buffer）
  }
  ComFrame f;
  f.type = bytes[0];
  f.apid = static_cast<uint16_t>((static_cast<uint16_t>(bytes[2]) << 8) | bytes[3]);
  f.len = static_cast<uint32_t>(len);
  std::memcpy(f.data, bytes, len);
  Os::ScopeLock lock(m_coreLock);
  (void)m_core.enqueue(static_cast<ComPriority>(portNum), f);  // 滿了 Core 計數＋節流事件
}

void ComQueue::comStatusIn_handler(FwIndexType, Fw::Success& condition) {
  m_linkBusy = (condition != Fw::Success::SUCCESS);
}

void ComQueue::run_handler(FwIndexType, U32) {
  bool have = false;
  {
    Os::ScopeLock lock(m_coreLock);
    have = (m_core.runEmit(m_linkBusy, &m_txFrame, 1) == 1);
  }
  U8 sent = 0;
  while (have) {
    sendFrame();
    ++sent;
    Os::ScopeLock lock(m_coreLock);
    have = (sent < m_core.maxPktsPerRun()) && m_core.nextFrame(m_linkBusy, m_txFrame);
  }
  if (++m_tlmDiv >= kTlmEveryTicks) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

void ComQueue::sendFrame() {
  if (!this->isConnected_buffOut_OutputPort(0)) return;
  Fw::Buffer b(m_txFrame.data, static_cast<Fw::Buffer::SizeType>(m_txFrame.len));
  ComCfg::FrameContext ctx;  // 本鏈的標頭欄位都在 LogicalPacket 裡，context 用預設值
  this->buffOut_out(0, b, ctx);
}

void ComQueue::writeTlm() {
  Os::ScopeLock lock(m_coreLock);
  this->tlmWrite_QueueHwmP0(m_core.hwm(ComPriority::P0));
  this->tlmWrite_QueueHwmP1(m_core.hwm(ComPriority::P1));
  this->tlmWrite_QueueHwmP2(m_core.hwm(ComPriority::P2));
  this->tlmWrite_QueueHwmP3(m_core.hwm(ComPriority::P3));
  this->tlmWrite_QueueHwmP4(m_core.hwm(ComPriority::P4));
  this->tlmWrite_DropCountTotal(m_core.dropCountTotal());
  this->tlmWrite_HighWater(m_core.highWater() ? 1 : 0);
}

void ComQueue::onQueueFullDrop(ComPriority p, uint32_t dropCount) {
  this->log_WARNING_LO_QueueFullDrop(static_cast<ComQueuePriority::T>(static_cast<U8>(p)), dropCount);
}

}  // namespace Components
