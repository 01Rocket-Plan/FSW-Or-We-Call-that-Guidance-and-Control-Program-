# BleDriver — Base ID 0x8800（提案，待首席核准）。Hardware model STM32WB55CGU6_INTERNAL_BLE。
# LoRa RfDriver（0x8400）的備用傳輸：埠集合與 RfDriver 相同，拓樸換實例即可；RfDriverCore 原檔共用、不改。
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
