// ======================================================================
// \title  WifiCredStore.hpp
// \author 無線組（Viktor）
// \brief  窄的 WiFi 憑證介面（SSID／密碼）
//
//   本介面只負責提供 SSID 與密碼緩衝。真機由平台從 NVS 或外部設定實作；
//   測試在測試檔裡提供明顯假值。
//   本頭檔不含任何 SSID 或密碼字串。密碼不得出現在日誌、事件或遙測。
// ======================================================================
#ifndef WIRELESS_WIFICREDSTORE_HPP
#define WIRELESS_WIFICREDSTORE_HPP

#include <cstddef>

namespace Components {

//! 憑證緩衝上限（SSID 32＋NUL，密碼 64＋NUL）
constexpr std::size_t kWifiSsidMax = 33;
constexpr std::size_t kWifiPassMax = 65;

class IWifiCredStore {
 public:
  virtual ~IWifiCredStore() = default;
  //! 取得 SSID。成功時寫入 buf（含 NUL），回傳 true。
  virtual bool getSsid(char* buf, std::size_t maxLen) = 0;
  //! 取得密碼。成功時寫入 buf（含 NUL），回傳 true。
  virtual bool getPassword(char* buf, std::size_t maxLen) = 0;
};

//! 空憑證（缺失）——init 會失敗，RfDriverCore 進 FAULT。
class EmptyWifiCredStore final : public IWifiCredStore {
 public:
  bool getSsid(char* buf, std::size_t maxLen) override {
    (void)buf;
    (void)maxLen;
    return false;
  }
  bool getPassword(char* buf, std::size_t maxLen) override {
    (void)buf;
    (void)maxLen;
    return false;
  }
};

}  // namespace Components

#endif
