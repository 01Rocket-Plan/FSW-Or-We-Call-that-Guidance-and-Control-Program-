# BleDriver — 藍牙備用傳輸（BLE），Base ID 0x8800。Sim 為 BleDriverSim 0x8880。
# 埠集合與 RfDriver（0x8400，LoRa 主鏈）相同，拓樸換實例即可；RfDriverCore 原檔共用、不改。
# 硬體型號：ESP32_INTERNAL_BLE（預設，初代 ESP32／BLE 4.2）或 STM32WB55CGU6_INTERNAL_BLE（BLE 5.4）。
# IBleHal 後端由平台實作：ESP-IDF NimBLE／Bluedroid，或 STM32_WPAN。
# 目前不放進任何部署拓樸（主鏈仍是 LoRa）；選用時在部署的 instances.fpp 把 rfDriver 換成 bleDriver。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/BleDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/BleLinkCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/BleDriver.cpp"
)
set(MOD_DEPS RocketTypes Components_Wireless_Ports Components_Wireless_RfDriver)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest）；-DHOST_TEST_MAIN 版本見 run_host_tests.sh
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/BleDriverHostTest.cpp")
set(UT_MOD_DEPS Components_Wireless_BleDriverSim Components_Wireless_Framer)
register_fprime_ut()
