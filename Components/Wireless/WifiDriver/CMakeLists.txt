# WifiDriver — WiFi 近距離備用傳輸，Base ID 0x8900。Sim 為 WifiDriverSim 0x8980。
# 埠集合與 RfDriver（0x8400，LoRa 主鏈）相同，拓樸換實例即可；RfDriverCore 原檔共用、不改。
# 硬體型號：ESP32_INTERNAL_WIFI（預設，初代 ESP32）或 UNSUPPORTED（無 WiFi 的 MCU，init 失敗）。
# IWifiHal 後端由平台實作：ESP-IDF esp_wifi + lwIP UDP。
# 目前不放進任何部署拓樸（主鏈仍是 LoRa）；選用時在部署的 instances.fpp 增加實例。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/WifiDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/WifiLinkCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/WifiDriver.cpp"
)
set(MOD_DEPS RocketTypes Components_Wireless_Ports Components_Wireless_RfDriver)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest）；-DHOST_TEST_MAIN 版本見 run_host_tests.sh
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/WifiDriverHostTest.cpp")
set(UT_MOD_DEPS Components_Wireless_WifiDriverSim Components_Wireless_Framer)
register_fprime_ut()
