# RfDriverSim — Base ID 0x8480. Same ports as RfDriver; no hardware model/role.
# TODO(無線組)：F´ 外殼 RfDriverSim.hpp／.cpp（繼承 RfDriverSimComponentBase、呼叫 Core）— PR #31 合併後補（issue #37-3）。
# 外殼還沒寫之前，模組照樣用真 F´ 建：FPP 自動碼＋Core 都會編譯，只是還不能放進拓樸。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/RfDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/LoraTimeOnAir.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RfDriverSimDevice.cpp"
)
set(MOD_DEPS RocketTypes Components_Wireless_Ports Components_Wireless_RfDriver)
register_fprime_module()

# 單元測試：LoRa 空中時間（Semtech 公式）＋ RfDriverSimDevice 佔線模型（issue #60）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/LoraTimeOnAirTest.cpp")
register_fprime_ut()
