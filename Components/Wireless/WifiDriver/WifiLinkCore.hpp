// ======================================================================
// \title  WifiLinkCore.hpp
// \author 無線組（Viktor）
// \brief  WiFi 傳輸層核心：把 RfDriverCore 的 IRfBus（整幀寫／位元組讀）接到 UDP。
//
//   定位：LoRa（RfDriver，0x8400）的「WiFi 近距離備用傳輸」。主線 MCU 為初代
//   ESP32（ESP32 DevKit U1，內建 2.4 GHz WiFi）；備用 MCU STM32WB55CGU6 沒有
//   WiFi，型號 UNSUPPORTED 時 init 失敗。上層（Framer／ComQueue／既有上行鑑權）
//   完全不變：
//     ComQueue → Framer 幀 → RfDriverCore（同一份，不改）→ WifiLinkCore（本檔）
//       → IWifiHal → 平台 WiFi 堆疊（ESP-IDF esp_wifi + lwIP UDP）
//
//   承載：一個 UDP 資料報恰好一幀，不加私有切片頭（幀上限對齊 RfDriverCore
//   的 512 B；正式 Framer 幀 ≤255 B）。超長資料報整包丟、計數，不交半幀。
//
//   掉線處理（對齊下行契約：不重傳、丟了就計數、永不阻塞）：
//     - 沒有關聯／沒有對端 → writeFrame 回 0，RfDriverCore 記 TX_FAIL，
//       本層另記 noLinkDrops。不阻塞、不排隊等重連。
//     - HAL／socket 暫時無法送出 → 本幀留在 pending，下一拍 service() 再試；
//       未送完前新幀回 0（忙，丟，計 busyDrops）。外殼 popFrame 先看 txPending()。
//     - 關聯中斷 → 送到一半的幀丟掉並通知 Core（takeTxAborted）；不跨關聯拼接。
//     - ready()＝協定堆疊活著（不是「已關聯」）：未關聯是常態，不能讓
//       RfDriverCore 因此進 FAULT。
//
//   安全：SSID／密碼只從 IWifiCredStore（NVS／外部設定或測試假值）載入；
//   本層不實作、不繞過上行鑑權。密碼不寫進日誌。
//
//   純 C++14，不 include 任何平台 HAL，不 sleep、不讀系統時間；主機可測。
// ======================================================================
#ifndef WIRELESS_WIFILINKCORE_HPP
#define WIRELESS_WIFILINKCORE_HPP

#include <cstdint>

#include "Components/Wireless/RfDriver/RfDriverCore.hpp"
#include "Components/Wireless/WifiDriver/IWifiHal.hpp"
#include "Components/Wireless/WifiDriver/WifiCredStore.hpp"

namespace Components {

//! 一幀最大長度（與 RfDriverCore 的 m_frameBuf 一致）
constexpr uint32_t kWifiMaxFrame = 512u;
//! ESP32 WiFi TX 功率合理範圍（dBm）；HAL 可視板級限制再夾
constexpr int8_t kWifiTxPowerMinEsp32_dBm = 8;
constexpr int8_t kWifiTxPowerMaxEsp32_dBm = 20;

//! 把設定值夾在該模式能接受的範圍。UNSUPPORTED 型號維持原樣（init 會失敗）。
WifiRadioConfig clampWifiConfig(const WifiRadioConfig& in);

//! 同時是 RfDriverCore 的 IRfTxDoneSource：WiFi 不套 LoRa 空中時間，
//! UDP 送出成功即視為 TxDone。
class WifiLinkCore final : public IRfBus, public IRfTxDoneSource {
 public:
  WifiLinkCore(IWifiHal& hal, IWifiCredStore& creds);

  void configure(const WifiRadioConfig& cfg) { m_cfg = clampWifiConfig(cfg); }
  const WifiRadioConfig& config() const { return m_cfg; }

  // ---- IRfBus（RfDriverCore 呼叫）
  bool init() override;
  int32_t writeFrame(const uint8_t* data, uint32_t len) override;
  int32_t readBytes(uint8_t* data, uint32_t maxLen) override;
  bool selftestLoopback() override;
  bool ready() override;

  // ---- IRfTxDoneSource
  bool takeTxDone() override;
  bool takeTxAborted() override;
  void abortTx() override;
  bool usesLoraAirtime() const override { return false; }

  //! 每拍呼叫一次（外殼在 RfDriverCore.tick 之前）：追蹤關聯變化、續送、收包。
  void service();

  // ---- 遙測
  bool linkUp() const { return m_linkUp; }
  uint8_t channel() const { return m_cfg.channel; }
  int8_t txPower() const { return m_cfg.txPower_dBm; }
  int8_t rssi() { return m_hal.rssi_dBm(); }
  uint32_t linkUpCount() const { return m_linkUpCount; }
  uint32_t linkDownCount() const { return m_linkDownCount; }
  uint32_t noLinkDrops() const { return m_noLinkDrops; }
  uint32_t busyDrops() const { return m_busyDrops; }
  uint32_t txAbortedOnLinkLoss() const { return m_txAbortedOnLinkLoss; }
  uint32_t framesFullySent() const { return m_framesFullySent; }
  uint32_t rxFrames() const { return m_rxFrames; }
  uint32_t rxOverflowDrops() const { return m_rxOverflowDrops; }
  bool txPending() const { return m_txLen > 0; }
  uint32_t txAbortedOnTimeout() const { return m_txAbortedOnTimeout; }

  //! 供測試直接驗證夾取
  static uint8_t clampChannel(uint8_t ch, WifiHalMode mode);
  static int8_t clampTxPower(int8_t dBm);

 private:
  void flushTx();
  void pollRx();
  void onLinkChange(bool up);
  bool enqueueRx(const uint8_t* data, uint32_t len);
  bool fillCreds();

  IWifiHal& m_hal;
  IWifiCredStore& m_creds;
  WifiRadioConfig m_cfg{};
  bool m_started = false;
  bool m_linkUp = false;

  uint8_t m_txBuf[kWifiMaxFrame]{};
  uint32_t m_txLen = 0;
  bool m_txDoneFlag = false;
  bool m_txAbortedFlag = false;

  static constexpr uint32_t kRxFifoCap = 1024u;
  uint8_t m_rxFifo[kRxFifoCap]{};
  uint32_t m_rxFifoRd = 0;
  uint32_t m_rxFifoCount = 0;

  uint32_t m_linkUpCount = 0;
  uint32_t m_linkDownCount = 0;
  uint32_t m_noLinkDrops = 0;
  uint32_t m_busyDrops = 0;
  uint32_t m_txAbortedOnLinkLoss = 0;
  uint32_t m_txAbortedOnTimeout = 0;
  uint32_t m_framesFullySent = 0;
  uint32_t m_rxFrames = 0;
  uint32_t m_rxOverflowDrops = 0;
};

}  // namespace Components

#endif
