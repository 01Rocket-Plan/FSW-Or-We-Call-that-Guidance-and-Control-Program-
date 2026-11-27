// ======================================================================
// \title  IWifiHal.hpp
// \author 無線組（Viktor）
// \brief  WiFi HAL 介面（非阻塞、可注入）
//
//   WifiLinkCore 不得 #include 任何 ESP-IDF 或平台 HAL 頭檔；本介面是唯一
//   平台邊界。真機由平台以 esp_wifi + lwIP UDP 實作；測試用 WifiHalSim。
//   堆疊回呼只入列，在 run／service 時輪詢。
// ======================================================================
#ifndef WIRELESS_IWIFIHAL_HPP
#define WIRELESS_IWIFIHAL_HPP

#include <cstdint>

namespace Components {

//! 工作模式（與 FPP WifiMode 數值對齊）
enum class WifiHalMode : uint8_t { STA = 0, SOFT_AP = 1 };

//! 硬體型號（與 FPP WifiHwModel 數值對齊）
enum class WifiHwKind : uint8_t {
  UNSUPPORTED = 0,
  ESP32_INTERNAL_WIFI = 1,
};

//! 交給 HAL 的無線設定（來自 prmDb＋憑證介面；不含飛行金鑰）
struct WifiRadioConfig {
  WifiHwKind hwModel = WifiHwKind::ESP32_INTERNAL_WIFI;
  WifiHalMode mode = WifiHalMode::STA;
  uint8_t channel = 0;       //!< 0＝交給堆疊／AP；SoftAP 強制 1–13
  int8_t txPower_dBm = 20;   //!< 由 clampWifiConfig 夾到 8–20；HAL 可視板級再夾
  uint16_t localPort = 4242;
  uint32_t peerIp = 0;       //!< 網路位元序：31–24＝第一段，例如 127.0.0.1＝0x7F000001
  uint16_t peerPort = 4242;
  bool learnPeerFromRx = false;
  char ssid[33] = {0};
  char password[65] = {0};
};

//! WiFi HAL。全部非阻塞。
class IWifiHal {
 public:
  virtual ~IWifiHal() = default;

  //! 依設定啟動 STA 或 SoftAP，建立 UDP socket。失敗回 false。
  virtual bool stackStart(const WifiRadioConfig& cfg) = 0;
  //! 協定堆疊是否仍在運作（不是「已關聯」）
  virtual bool stackAlive() = 0;
  //! 無副作用自檢（例如讀 MAC）
  virtual bool selftest() = 0;
  //! 鏈路是否可用：STA 已關聯且有對端；SoftAP 至少一個客戶端且有對端
  virtual bool linkUp() = 0;
  //! 對目前對端送一個資料報。緩衝滿或未就緒回 false（本資料尚未送出）。
  virtual bool send(const uint8_t* data, uint32_t len) = 0;
  //! 非阻塞取一個資料報。沒有回 0。
  //! 若資料報比 maxLen 長：消耗掉該包、不越界寫入，並回傳實際長度（＞maxLen），
  //! 讓上層計入溢位、不交半幀。learnPeerFromRx 時可在此更新對端。
  virtual int32_t pollRecv(uint8_t* data, uint32_t maxLen) = 0;
  //! 可選：RSSI（dBm）。沒有就回 0。
  virtual int8_t rssi_dBm() { return 0; }
};

}  // namespace Components

#endif
