// ======================================================================
// \title  RfDriverSim.cpp
// \brief  rfDriverSim F´ 外殼：run → Device.advance＋Core.tick；dataIn → TX 槽；Core 回呼 → 事件／遙測／埠。
// ======================================================================
#include "Components/Wireless/RfDriverSim/RfDriverSim.hpp"

#include <cstring>
#include <new>

#include "Components/Wireless/RfDriver/RfDriverCore.hpp"
#include "Components/Wireless/RfDriverSim/RfDriverSimDevice.hpp"

namespace Components {

namespace {

RocketTypes::DrvState toFpp(DrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

}  // namespace

//! Core＋Device＋1 格 TX 槽；回呼轉給元件的 F´ 埠
class RfDriverSimImpl final : private IRfSink, private IRfTxQueue {
 public:
  explicit RfDriverSimImpl(RfDriverSim& owner) : m_owner(owner), m_txLen(0), m_txSlot{}, m_rxBuf{}, m_dev(), m_core(m_dev, *this, *this) {}

  //! dataIn：槽滿（上一幀 Core 還沒取走）→ 拒收、回 false
  bool offer(const U8* data, U32 len) {
    if (data == nullptr || len == 0 || len > sizeof(m_txSlot) || m_txLen != 0) return false;
    std::memcpy(m_txSlot, data, len);
    m_txLen = len;
    return true;
  }

  // 宣告順序＝建構順序：m_core 建構時要用到 m_dev
 private:
  RfDriverSim& m_owner;
  uint32_t m_txLen;
  uint8_t m_txSlot[256];
  uint8_t m_rxBuf[256];

 public:
  RfDriverSimDevice m_dev;
  RfDriverCore m_core;

 private:
  // ---- IRfTxQueue
  uint32_t popFrame(uint8_t* buf, uint32_t maxLen) override {
    if (m_txLen == 0 || m_txLen > maxLen) return 0;
    std::memcpy(buf, m_txSlot, m_txLen);
    const uint32_t n = m_txLen;
    m_txLen = 0;
    return n;
  }

  // ---- IRfSink
  void onStateChange(DrvState from, DrvState to) override {
    m_owner.log_ACTIVITY_HI_RfState(toFpp(from), toFpp(to));
    m_owner.tlmWrite_State(toFpp(to));
    if (m_owner.isConnected_healthOut_OutputPort(0)) {
      m_owner.healthOut_out(0, toFpp(to));
    }
  }

  void onEvent(RfEvent ev, uint32_t detail) override {
    switch (ev) {
      case RfEvent::TX_TIMEOUT:
        m_owner.log_WARNING_LO_RfTxTimeout(detail);
        break;
      case RfEvent::FAULT:
        m_owner.log_WARNING_HI_RfFault(detail);
        break;
      default:
        break;  // INIT_OK／SELFTEST_OK／READY 由 RfState 事件表示；TX_FAIL 走 TxDroppedCount 遙測
    }
  }

  void onUplink(const uint8_t* data, uint32_t len) override {
    if (!m_owner.isConnected_dataOut_OutputPort(0) || len == 0) return;
    const uint32_t n = (len < sizeof(m_rxBuf)) ? len : static_cast<uint32_t>(sizeof(m_rxBuf));
    std::memcpy(m_rxBuf, data, n);
    Fw::Buffer b(m_rxBuf, static_cast<Fw::Buffer::SizeType>(n));
    ComCfg::FrameContext ctx;
    m_owner.dataOut_out(0, b, ctx);
  }

  void onComStatus(bool ready) override {
    // 槽裡還有幀沒送 → 對 comQueue 仍是忙碌
    const bool linkReady = ready && (m_txLen == 0);
    if (m_owner.isConnected_comStatusOut_OutputPort(0)) {
      Fw::Success s = linkReady ? Fw::Success::SUCCESS : Fw::Success::FAILURE;
      m_owner.comStatusOut_out(0, s);
    }
  }
};

static_assert(sizeof(RfDriverSimImpl) <= RfDriverSim::kImplBytes, "RfDriverSim::kImplBytes 太小");
static_assert(alignof(RfDriverSimImpl) <= alignof(std::max_align_t), "RfDriverSimImpl 對齊");

RfDriverSim::RfDriverSim(const char* compName)
    : RfDriverSimComponentBase(compName), m_implStorage{}, m_paramsLoaded(false), m_started(false), m_tlmDiv(0) {
  new (m_implStorage) RfDriverSimImpl(*this);
}

RfDriverSim::~RfDriverSim() { impl().~RfDriverSimImpl(); }

RfDriverSimImpl& RfDriverSim::impl() { return *reinterpret_cast<RfDriverSimImpl*>(m_implStorage); }

F64 RfDriverSim::nowSeconds() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

// F´ 開機逐個呼叫 parameterUpdated（其他參數還沒載完）→ 等 parametersLoaded 一次套
void RfDriverSim::parameterUpdated(FwPrmIdType) {
  if (m_paramsLoaded) {
    loadParams();
  }
}
void RfDriverSim::parametersLoaded() {
  m_paramsLoaded = true;
  loadParams();
}

void RfDriverSim::loadParams() {
  Fw::ParamValid v;
  RfParams p;
  p.channel = this->paramGet_channel(v);
  p.power_dBm = this->paramGet_power_dBm(v);
  p.txTimeout_s = this->paramGet_txTimeout_s(v);
  p.initDeadline_s = this->paramGet_initDeadline_s(v);
  p.selftestDeadline_s = this->paramGet_selftestDeadline_s(v);
  p.readyDeadline_s = this->paramGet_readyDeadline_s(v);
  p.txTimeoutLimit = this->paramGet_txTimeoutLimit(v);
  // LoRa 數據機（issue #60）：Core（等空中時間才送下一包）和 Device（佔線）用同一份參數
  p.spreadingFactor = this->paramGet_loraSf(v);
  if (!loraBwFromHz(this->paramGet_loraBw_Hz(v), p.bandwidth)) {
    p.bandwidth = static_cast<LoraBw>(0xFF);  // 不在 SX1278 頻寬表 → rfParamsValid 為 false → start() 進 FAULT
  }
  p.codingRate = static_cast<LoraCr>(this->paramGet_loraCr(v));
  p.preambleLen = this->paramGet_loraPreambleLen(v);
  p.implicitHeader = !this->paramGet_loraExplicitHeader(v);
  p.crcOn = this->paramGet_loraCrcOn(v);
  const U8 ldro = this->paramGet_loraLdro(v);
  p.lowDataRateOptimize = (ldro == 0) ? LoraLdro::OFF : ((ldro == 1) ? LoraLdro::ON : LoraLdro::AUTO);
  // tickPeriod_us 要跟 run 掛的 rate group 一致：DsMcu 掛 rg100Hz → 10000 µs（FPP 預設）
  p.tickPeriod_us = this->paramGet_tickPeriod_us(v);
  RfDriverSimImpl& im = impl();
  im.m_core.configure(p);

  im.m_dev.setDropRate(this->paramGet_dropRate(v));
  im.m_dev.setLatencyTicks(this->paramGet_latencyTicks(v));
  im.m_dev.setBer(this->paramGet_ber(v));
  im.m_dev.configureFrom(p);
}

void RfDriverSim::dataIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext&) {
  RfDriverSimImpl& im = impl();
  // 收下 → 槽佔用中，對 comQueue 回報忙碌，等 Core 取走、送完再回報就緒。
  // 拒收（槽滿）→ 照合約 §4「永不阻塞」直接丟；comQueue 只在就緒時送，正常不會發生。
  (void)im.offer(data.getData(), static_cast<U32>(data.getSize()));
  if (this->isConnected_comStatusOut_OutputPort(0)) {
    Fw::Success s = Fw::Success::FAILURE;
    this->comStatusOut_out(0, s);
  }
}

void RfDriverSim::run_handler(FwIndexType, U32) {
  RfDriverSimImpl& im = impl();
  const F64 now_s = nowSeconds();
  if (!m_started) {
    // 參數（prmDb）在 loadParameters 時已套；第一次 run 才開機握手（OFF → INIT）
    m_started = true;
    im.m_core.start();
  }
  im.m_dev.advance();
  im.m_core.tick(now_s);
  if (++m_tlmDiv >= kTlmEveryTicks) {
    m_tlmDiv = 0;
    this->tlmWrite_State(toFpp(im.m_core.state()));
    this->tlmWrite_TxTimeoutCount(im.m_core.txTimeoutCount());
    this->tlmWrite_TxDroppedCount(im.m_core.txDroppedCount());
    this->tlmWrite_SentFrames(im.m_core.sentFrames());
    this->tlmWrite_SimDropCount(im.m_dev.framesDropped());
  }
}

}  // namespace Components
