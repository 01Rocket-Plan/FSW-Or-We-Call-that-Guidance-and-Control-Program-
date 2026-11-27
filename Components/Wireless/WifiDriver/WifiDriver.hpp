// ======================================================================
// \title  WifiDriver.hpp
// \author 無線組（Viktor）
// \brief  WiFi 近距離備用傳輸 F´ 外殼（LoRa RfDriver 的備用；主飛行鏈仍為 LoRa）。
//
//   邏輯全在 Core：RfDriverCore（握手／TX 逾時／FAULT，與 LoRa 同一份）
//   ＋WifiLinkCore（UDP 一幀一包／掉線）。本檔只搬資料。
//
//   HAL：拓樸在 start 前呼叫 setHal() 接平台實作（ESP-IDF esp_wifi＋lwIP UDP）；
//   沒接時用內建的「空 HAL」（stackStart 回 false）→ INIT 失敗 → FAULT。
//   憑證：setCredStore()；沒接時用空憑證 → INIT 失敗 → FAULT。
//
//   與 RfDriver／BleDriver 相同的已知限制：埠集合沒有 dataReturnOut，dataIn
//   的 Fw::Buffer 由本元件同步複製、不歸還。dataIn 與 run 必須掛在同一個
//   rate group（被動元件、佇列無鎖）。
// ======================================================================
#ifndef COMPONENTS_WIRELESS_WIFIDRIVER_HPP
#define COMPONENTS_WIRELESS_WIFIDRIVER_HPP

#include "Components/Wireless/RfDriver/RfDriverCore.hpp"
#include "Components/Wireless/WifiDriver/WifiCredStore.hpp"
#include "Components/Wireless/WifiDriver/WifiDriverComponentAc.hpp"
#include "Components/Wireless/WifiDriver/WifiLinkCore.hpp"

namespace Components {

class WifiDriver final : public WifiDriverComponentBase, private IRfSink, private IRfTxQueue {
 public:
  explicit WifiDriver(const char* compName);
  ~WifiDriver() override = default;

  //! 接真機 HAL（平台提供）；必須在第一個 run 之前呼叫
  void setHal(IWifiHal& hal) { m_halRef.target = &hal; }
  //! 接憑證庫（NVS／外部設定）；必須在第一個 run 之前呼叫
  void setCredStore(IWifiCredStore& creds) { m_credRef.target = &creds; }

  //! 每幾拍寫一次遙測。rg100Hz 下 100＝1 Hz
  static constexpr U32 kTlmEveryTicks = 100;
  static constexpr U32 kTxQueueDepth = 4;

 private:
  class HalRef final : public IWifiHal {
   public:
    IWifiHal* target = nullptr;
    bool stackStart(const WifiRadioConfig& c) override { return target != nullptr && target->stackStart(c); }
    bool stackAlive() override { return target != nullptr && target->stackAlive(); }
    bool selftest() override { return target != nullptr && target->selftest(); }
    bool linkUp() override { return target != nullptr && target->linkUp(); }
    bool send(const uint8_t* d, uint32_t n) override { return target != nullptr && target->send(d, n); }
    int32_t pollRecv(uint8_t* d, uint32_t n) override { return (target != nullptr) ? target->pollRecv(d, n) : 0; }
    int8_t rssi_dBm() override { return (target != nullptr) ? target->rssi_dBm() : 0; }
  };

  class CredRef final : public IWifiCredStore {
   public:
    IWifiCredStore* target = nullptr;
    EmptyWifiCredStore empty;
    bool getSsid(char* buf, std::size_t n) override {
      return (target != nullptr) ? target->getSsid(buf, n) : empty.getSsid(buf, n);
    }
    bool getPassword(char* buf, std::size_t n) override {
      return (target != nullptr) ? target->getPassword(buf, n) : empty.getPassword(buf, n);
    }
  };

  void dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;
  void run_handler(FwIndexType portNum, U32 context) override;

  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;
  void loadParams();

  void onStateChange(DrvState from, DrvState to) override;
  void onEvent(RfEvent ev, uint32_t detail) override;
  void onUplink(const uint8_t* data, uint32_t len) override;
  void onComStatus(bool ready) override;

  uint32_t popFrame(uint8_t* buf, uint32_t maxLen) override;

  F64 nowSeconds();
  void writeTlm();

  HalRef m_halRef;
  CredRef m_credRef;
  WifiLinkCore m_link;
  RfDriverCore m_core;
  bool m_started;
  bool m_paramsLoaded;
  bool m_lastLinkUp;
  bool m_loggedUnsupported;
  U32 m_tlmDiv;
  U32 m_restartCount;
  F64 m_faultSince_s;
  F64 m_faultRetry_s;

  U8 m_txFrames[kTxQueueDepth][kWifiMaxFrame];
  U32 m_txLens[kTxQueueDepth];
  U32 m_txRd;
  U32 m_txCount;
  U32 m_queueDrops;

  U8 m_rxBuf[256];
};

}  // namespace Components

#endif
