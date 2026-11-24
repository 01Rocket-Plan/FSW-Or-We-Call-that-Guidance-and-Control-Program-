// ======================================================================
// \title  BleDriver.hpp
// \author 無線組（Viktor）
// \brief  藍牙備用傳輸（BLE）F´ 外殼（LoRa RfDriver 的備用；主飛行鏈仍為 LoRa）。
//
//   邏輯全在 Core：RfDriverCore（握手／TX 逾時／FAULT，與 LoRa 同一份）＋BleLinkCore（切片／重組／掉線）。
//   本檔只搬資料：dataIn → 小佇列 → Core；Core 上行 → dataOut；狀態 → comStatusOut／healthOut／遙測。
//
//   HAL：拓樸在 start 前呼叫 setHal() 接平台實作（ESP-IDF NimBLE／Bluedroid，或 STM32_WPAN）；
//   沒接時用內建的「空 HAL」（stackStart 回 false）→ INIT 失敗 → FAULT，不會假裝有鏈路。
//
//   與 RfDriver 相同的已知限制：埠集合沒有 dataReturnOut，dataIn 的 Fw::Buffer 由本元件同步複製、
//   不歸還；之後兩個驅動一起補（接 F´ v4 ComQueue／Framer 的 buffer 歸還流程時）。
//
//   dataIn 與 run 必須掛在同一個 rate group（被動元件、佇列無鎖）。
// ======================================================================
#ifndef COMPONENTS_WIRELESS_BLEDRIVER_HPP
#define COMPONENTS_WIRELESS_BLEDRIVER_HPP

#include "Components/Wireless/BleDriver/BleDriverComponentAc.hpp"
#include "Components/Wireless/BleDriver/BleLinkCore.hpp"
#include "Components/Wireless/RfDriver/RfDriverCore.hpp"

namespace Components {

class BleDriver final : public BleDriverComponentBase, private IRfSink, private IRfTxQueue {
 public:
  explicit BleDriver(const char* compName);
  ~BleDriver() override = default;

  //! 接真機 HAL（平台提供：ESP-IDF 或 STM32_WPAN）；必須在第一個 run 之前呼叫
  void setHal(IBleHal& hal) { m_halRef.target = &hal; }

  //! 每幾拍寫一次遙測。rg100Hz 下 100＝1 Hz（對齊 TlmSend P3 診斷 1 Hz；P2 健康為 10 Hz）
  static constexpr U32 kTlmEveryTicks = 100;
  static constexpr U32 kTxQueueDepth = 4;

 private:
  //! 轉接到外部 HAL；沒接時一律失敗（見檔頭）
  class HalRef final : public IBleHal {
   public:
    IBleHal* target = nullptr;
    bool stackStart(const BleRadioConfig& c) override { return target != nullptr && target->stackStart(c); }
    bool stackAlive() override { return target != nullptr && target->stackAlive(); }
    bool selftest() override { return target != nullptr && target->selftest(); }
    bool linkUp() override { return target != nullptr && target->linkUp(); }
    uint16_t attMtu() override { return (target != nullptr) ? target->attMtu() : kBleAttMtuMin; }
    bool notify(const uint8_t* d, uint16_t n) override { return target != nullptr && target->notify(d, n); }
    int32_t pollWrite(uint8_t* d, uint16_t n) override { return (target != nullptr) ? target->pollWrite(d, n) : 0; }
  };

  // ---- 埠
  void dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;
  void run_handler(FwIndexType portNum, U32 context) override;

  // ---- 參數
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;
  void loadParams();

  // ---- IRfSink（Core → 外殼）
  void onStateChange(DrvState from, DrvState to) override;
  void onEvent(RfEvent ev, uint32_t detail) override;
  void onUplink(const uint8_t* data, uint32_t len) override;
  void onComStatus(bool ready) override;

  // ---- IRfTxQueue（Core 取下一幀）：忙碌時不 pop，避免擠塞時丟幀、SentFrames 對不上
  uint32_t popFrame(uint8_t* buf, uint32_t maxLen) override;

  F64 nowSeconds();
  void writeTlm();

  HalRef m_halRef;
  BleLinkCore m_link;
  RfDriverCore m_core;
  bool m_started;
  bool m_paramsLoaded;
  bool m_lastLinkUp;
  U32 m_tlmDiv;
  U32 m_restartCount;
  F64 m_faultSince_s;
  F64 m_faultRetry_s;

  // dataIn 小佇列（被動元件、同一 rate group 內呼叫，不需要鎖）
  U8 m_txFrames[kTxQueueDepth][kBleMaxFrame];
  U32 m_txLens[kTxQueueDepth];
  U32 m_txRd;
  U32 m_txCount;
  U32 m_queueDrops;

  //! 上行暫存：下游必須在呼叫內同步複製（本元件被動）
  U8 m_rxBuf[256];
};

}  // namespace Components

#endif
