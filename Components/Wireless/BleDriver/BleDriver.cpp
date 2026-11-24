// ======================================================================
// \title  BleDriver.cpp
// \brief  藍牙備用傳輸（BLE）F´ 外殼：每拍 BleLinkCore.service() → RfDriverCore.tick()；只搬資料。
// ======================================================================
#include "Components/Wireless/BleDriver/BleDriver.hpp"

namespace Components {

namespace {
RocketTypes::DrvState toFpp(DrvState s) { return static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)); }
}  // namespace

BleDriver::BleDriver(const char* compName)
    : BleDriverComponentBase(compName),
      m_halRef(),
      m_link(m_halRef),
      m_core(m_link, *this, *this),
      m_started(false),
      m_paramsLoaded(false),
      m_lastLinkUp(false),
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
  // BLE 不用 LoRa 空中時間：最後一片被 HAL 收下才算送完、斷線作廢直接丟、逾時時清掉半幀（Issue #60）
  m_core.setTxDoneSource(&m_link);
}

F64 BleDriver::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

// 開機載參數時 parameterUpdated 會逐個被呼叫 → 等 parametersLoaded 一次套；之後 GDS 改單一參數才即時套。
// 無線設定（功率／PHY／MTU…）在下一次 INIT 才送進 HAL／協定堆疊。
void BleDriver::parameterUpdated(FwPrmIdType) {
  if (m_paramsLoaded) {
    loadParams();
  }
}
void BleDriver::parametersLoaded() {
  m_paramsLoaded = true;
  loadParams();
}

void BleDriver::loadParams() {
  Fw::ParamValid v;
  BleRadioConfig c;
  const BleHwModel model = this->paramGet_model(v);
  if (model == BleHwModel::STM32WB55CGU6_INTERNAL_BLE) {
    c.hwModel = BleHwKind::STM32WB55CGU6_INTERNAL_BLE;
  } else {
    c.hwModel = BleHwKind::ESP32_INTERNAL_BLE;
  }
  const I32 pw = this->paramGet_txPower_dBm(v);
  c.txPower_dBm = static_cast<int8_t>((pw < -128) ? -128 : ((pw > 127) ? 127 : pw));
  c.phy = (this->paramGet_phy(v) == BlePhy::LE_2M) ? BlePhyMode::LE_2M : BlePhyMode::LE_1M;
  c.attMtu = this->paramGet_attMtu(v);
  c.advInterval_ms = this->paramGet_advInterval_ms(v);
  c.connInterval_ms = this->paramGet_connInterval_ms(v);
  c.supervisionTimeout_ms = this->paramGet_supervisionTimeout_ms(v);
  m_link.configure(c);  // 依型號夾功率；ESP32＋LE_2M 回退 LE_1M

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

void BleDriver::run_handler(FwIndexType, U32) {
  const F64 now = nowSeconds();
  if (!m_started) {
    if (!m_paramsLoaded) {
      loadParams();  // prmDb 沒接（SIL）→ 用 FPP 預設值
    }
    m_started = true;
    m_core.start();
  }
  m_link.service();
  const bool up = m_link.linkUp();
  if (up != m_lastLinkUp) {
    m_lastLinkUp = up;
    if (up) {
      this->log_ACTIVITY_LO_BleLinkUp(static_cast<U16>(m_halRef.attMtu()));
    } else {
      this->log_WARNING_LO_BleLinkDown(m_link.linkDownCount());
    }
  }
  m_core.tick(now);

  // FAULT 後隔 faultRetry_s 重新握手（合約 §4：FAULT 後由握手重新 INIT）
  if (m_core.state() == DrvState::FAULT && m_faultRetry_s > 0.0 && (now - m_faultSince_s) >= m_faultRetry_s) {
    ++m_restartCount;
    this->log_ACTIVITY_HI_BleRestart(m_restartCount);
    m_core.start();
  }

  if (++m_tlmDiv >= kTlmEveryTicks) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

void BleDriver::dataIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext&) {
  const U32 n = static_cast<U32>(data.getSize());
  if (data.getData() == nullptr || n == 0u || n > kBleMaxFrame || m_txCount >= kTxQueueDepth) {
    ++m_queueDrops;  // 永不阻塞：滿了就丟（合約 §1-2）
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

uint32_t BleDriver::popFrame(uint8_t* buf, uint32_t maxLen) {
  // 上一幀切片還沒送完就不要出佇列，避免擠塞時丟幀、SentFrames 對不上
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

void BleDriver::onStateChange(DrvState from, DrvState to) {
  if (to == DrvState::FAULT) {
    m_faultSince_s = nowSeconds();
  }
  this->log_ACTIVITY_HI_RfState(toFpp(from), toFpp(to));
  if (this->isConnected_healthOut_OutputPort(0)) {
    this->healthOut_out(0, toFpp(to));
  }
}

void BleDriver::onEvent(RfEvent ev, uint32_t detail) {
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
      // TX_FAIL（沒連線／忙）只計數不發事件：BLE 沒人連是常態，避免事件洪水
      break;
  }
}

void BleDriver::onUplink(const uint8_t* data, uint32_t len) {
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

void BleDriver::onComStatus(bool ready) {
  if (this->isConnected_comStatusOut_OutputPort(0)) {
    Fw::Success s = ready ? Fw::Success::SUCCESS : Fw::Success::FAILURE;
    this->comStatusOut_out(0, s);
  }
}

void BleDriver::writeTlm() {
  Fw::ParamValid v;
  this->tlmWrite_State(toFpp(m_core.state()));
  this->tlmWrite_TxTimeoutCount(m_core.txTimeoutCount());
  this->tlmWrite_TxDroppedCount(m_core.txDroppedCount() + m_queueDrops);
  this->tlmWrite_SentFrames(m_core.sentFrames());
  this->tlmWrite_Model(this->paramGet_model(v));
  this->tlmWrite_Role(this->paramGet_role(v));
  this->tlmWrite_LinkUp(m_link.linkUp());
  this->tlmWrite_AttMtu(static_cast<U16>(m_halRef.attMtu()));
  this->tlmWrite_LinkDownCount(m_link.linkDownCount());
  this->tlmWrite_NoLinkDrops(m_link.noLinkDrops());
  this->tlmWrite_BusyDrops(m_link.busyDrops());
  this->tlmWrite_TxAbortedOnLinkLoss(m_link.txAbortedOnLinkLoss());
  this->tlmWrite_FragmentsSent(m_link.fragmentsSent());
  this->tlmWrite_RxFrames(m_link.rxFrames());
  this->tlmWrite_RxFragErrors(m_link.rxFragErrors());
  this->tlmWrite_RestartCount(m_restartCount);
  this->tlmWrite_RxOverflowDrops(m_link.rxOverflowDrops());
}

}  // namespace Components
