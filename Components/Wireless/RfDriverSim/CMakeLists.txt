# RfDriverSim — Base ID 0x8480. Same ports as RfDriver; no hardware model/role.
# F´ 外殼 RfDriverSim.hpp／.cpp（繼承 RfDriverSimComponentBase、呼叫 Core）；DsMcu 拓樸 rg100Hz（Issue #60）。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/RfDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/RfDriverSimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RfDriverSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Wireless_Ports Components_Wireless_RfDriver)
register_fprime_module()

# 單元測試：LoRa 空中時間（Semtech 公式）＋ RfDriverSimDevice 佔線模型（issue #60）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/LoraTimeOnAirTest.cpp")
register_fprime_ut()
