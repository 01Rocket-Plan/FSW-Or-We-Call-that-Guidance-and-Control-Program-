// ======================================================================
// \title  WifiDriver.cpp
// \brief  WiFi 近距離備用傳輸 F´ 外殼：每拍 WifiLinkCore.service() → RfDriverCore.tick()。
// ======================================================================
#include "Components/Wireless/WifiDriver/WifiDriver.hpp"

namespace Components {

namespace {
RocketTypes::DrvState toFpp(DrvState s) { return static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)); }
}  // namespace

WifiDriver::WifiDriver(const char* compName)
    : WifiDriverComponentBase(compName),
      m_halRef(),
      m_credRef(),
      m_link(m_halRef, m_credRef),
      m_core(m_link, *this, *this),
      m_started(false),
      m_paramsLoaded(false),
      m_lastLinkUp(false),
      m_loggedUnsupported(false),
      m_tlmDiv(0),
      m_restartCount(0),
      m_faultSince_s(0.0),
      m_faultRetry_s(5.0),
      m_txFrames{},
      m_txLens{},
      m_txRd(0),
      m_txCount(0),
      m_queueDrops(0),
      m_rxBuf{} {
  // WiFi 不用 LoRa 空中時間：UDP 送出成功才算送完（Issue #60 的非 LoRa 路徑）
  m_core.setTxDoneSource(&m_link);
}

F64 WifiDriver::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void WifiDriver::parameterUpdated(FwPrmIdType) {
  if (m_paramsLoaded) {
    loadParams();
  }
}
void WifiDriver::parametersLoaded() {
  m_paramsLoaded = true;
  loadParams();
}

void WifiDriver::loadParams() {
  Fw::ParamValid v;
  WifiRadioConfig c;
  const WifiHwModel model = this->paramGet_model(v);
  c.hwModel = (model == WifiHwModel::ESP32_INTERNAL_WIFI) ? WifiHwKind::ESP32_INTERNAL_WIFI
                                                          : WifiHwKind::UNSUPPORTED;
  c.mode = (this->paramGet_mode(v) == WifiMode::SOFT_AP) ? WifiHalMode::SOFT_AP : WifiHalMode::STA;
  c.channel = this->paramGet_channel(v);
  const I32 pw = this->paramGet_txPower_dBm(v);
  c.txPower_dBm = static_cast<int8_t>((pw < -128) ? -128 : ((pw > 127) ? 127 : pw));
  c.localPort = this->paramGet_localPort(v);
  c.peerIp = this->paramGet_peerIp(v);
  c.peerPort = this->paramGet_peerPort(v);
  c.learnPeerFromRx = this->paramGet_learnPeerFromRx(v);
  m_link.configure(c);

  RfParams p;
  p.power_dBm = static_cast<int32_t>(m_link.config().txPower_dBm);
  p.txTimeout_s = this->paramGet_txTimeout_s(v);
  p.initDeadline_s = this->paramGet_initDeadline_s(v);
  p.selftestDeadline_s = this->paramGet_selftestDeadline_s(v);
  p.readyDeadline_s = this->paramGet_readyDeadline_s(v);
  p.txTimeoutLimit = this->paramGet_txTimeoutLimit(v);
  m_core.configure(p);
  m_faultRetry_s = this->paramGet_faultRetry_s(v);
}

void WifiDriver::run_handler(FwIndexType, U32) {
  const F64 now = nowSeconds();
  if (!m_started) {
    if (!m_paramsLoaded) {
      loadParams();  // prmDb 沒接（SIL）→ 用 FPP 預設值
    }
    Fw::ParamValid v;
    if (this->paramGet_model(v) != WifiHwModel::ESP32_INTERNAL_WIFI && !m_loggedUnsupported) {
      this->log_WARNING_HI_WifiUnsupportedHw();
      m_loggedUnsupported = true;
    }
    m_started = true;
    m_core.start();
  }
  m_link.service();
  const bool up = m_link.linkUp();
  if (up != m_lastLinkUp) {
    m_lastLinkUp = up;
    if (up) {
      this->log_ACTIVITY_LO_WifiLinkUp();
    } else {
      this->log_WARNING_LO_WifiLinkDown(m_link.linkDownCount());
    }
  }
  m_core.tick(now);

  if (m_core.state() == DrvState::FAULT && m_faultRetry_s > 0.0 && (now - m_faultSince_s) >= m_faultRetry_s) {
    ++m_restartCount;
    this->log_ACTIVITY_HI_WifiRestart(m_restartCount);
    m_core.start();
  }

  if (++m_tlmDiv >= kTlmEveryTicks) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

void WifiDriver::dataIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext&) {
  const U32 n = static_cast<U32>(data.getSize());
  if (data.getData() == nullptr || n == 0u || n > kWifiMaxFrame || m_txCount >= kTxQueueDepth) {
    ++m_queueDrops;
    return;
  }
  const U32 slot = (m_txRd + m_txCount) % kTxQueueDepth;
  const U8* src = data.getData();
  for (U32 i = 0; i < n; ++i) {
    m_txFrames[slot][i] = src[i];
  }
  m_txLens[slot] = n;
  ++m_txCount;
}

uint32_t WifiDriver::popFrame(uint8_t* buf, uint32_t maxLen) {
  if (m_link.txPending()) {
    return 0u;
  }
  if (m_txCount == 0u) {
    return 0u;
  }
  const U32 n = m_txLens[m_txRd];
  const U32 slot = m_txRd;
  m_txRd = (m_txRd + 1u) % kTxQueueDepth;
  --m_txCount;
  if (n > maxLen) {
    ++m_queueDrops;
    return 0u;
  }
  for (U32 i = 0; i < n; ++i) {
    buf[i] = m_txFrames[slot][i];
  }
  return n;
}

void WifiDriver::onStateChange(DrvState from, DrvState to) {
  if (to == DrvState::FAULT) {
    m_faultSince_s = nowSeconds();
  }
  this->log_ACTIVITY_HI_RfState(toFpp(from), toFpp(to));
  if (this->isConnected_healthOut_OutputPort(0)) {
    this->healthOut_out(0, toFpp(to));
  }
}

void WifiDriver::onEvent(RfEvent ev, uint32_t detail) {
  switch (ev) {
    case RfEvent::INIT_OK:
      this->log_ACTIVITY_LO_InitOk();
      break;
    case RfEvent::SELFTEST_OK:
      this->log_ACTIVITY_LO_SelftestOk();
      break;
    case RfEvent::SELFTEST_FAIL:
      this->log_WARNING_HI_SelftestFail(detail);
      break;
    case RfEvent::TX_TIMEOUT:
      this->log_WARNING_LO_RfTxTimeout(detail);
      break;
    case RfEvent::FAULT:
      this->log_WARNING_HI_RfFault(detail);
      break;
    default:
      break;
  }
}

void WifiDriver::onUplink(const uint8_t* data, uint32_t len) {
  if (!this->isConnected_dataOut_OutputPort(0) || len == 0u) {
    return;
  }
  const U32 n = (len > sizeof(m_rxBuf)) ? static_cast<U32>(sizeof(m_rxBuf)) : len;
  for (U32 i = 0; i < n; ++i) {
    m_rxBuf[i] = data[i];
  }
  Fw::Buffer b(m_rxBuf, static_cast<Fw::Buffer::SizeType>(n));
  ComCfg::FrameContext ctx;
  this->dataOut_out(0, b, ctx);
}

void WifiDriver::onComStatus(bool ready) {
  if (this->isConnected_comStatusOut_OutputPort(0)) {
    Fw::Success s = ready ? Fw::Success::SUCCESS : Fw::Success::FAILURE;
    this->comStatusOut_out(0, s);
  }
}

void WifiDriver::writeTlm() {
  Fw::ParamValid v;
  this->tlmWrite_State(toFpp(m_core.state()));
  this->tlmWrite_TxTimeoutCount(m_core.txTimeoutCount());
  this->tlmWrite_TxDroppedCount(m_core.txDroppedCount() + m_queueDrops);
  this->tlmWrite_SentFrames(m_core.sentFrames());
  this->tlmWrite_Model(this->paramGet_model(v));
  this->tlmWrite_Role(this->paramGet_role(v));
  this->tlmWrite_WifiMode(this->paramGet_mode(v));
  this->tlmWrite_LinkUp(m_link.linkUp());
  this->tlmWrite_LinkDownCount(m_link.linkDownCount());
  this->tlmWrite_NoLinkDrops(m_link.noLinkDrops());
  this->tlmWrite_BusyDrops(m_link.busyDrops());
  this->tlmWrite_TxAbortedOnLinkLoss(m_link.txAbortedOnLinkLoss());
  this->tlmWrite_RxFrames(m_link.rxFrames());
  this->tlmWrite_RestartCount(m_restartCount);
  this->tlmWrite_RxOverflowDrops(m_link.rxOverflowDrops());
  this->tlmWrite_Channel(m_link.channel());
  this->tlmWrite_Rssi_dBm(m_link.rssi());
}

}  // namespace Components
