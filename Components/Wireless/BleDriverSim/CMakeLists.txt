# BleDriverSim — 藍牙備用傳輸（BLE）Sim，Base ID 0x8880（＝0x8800+0x80）。
# 埠集合與 BleDriver／RfDriver 相同，不宣告硬體型號／角色。
# 與 RfDriverSim 相同做法：先有 FPP 元件＋可測的 BleHalSim（IBleHal 模擬）；F´ 外殼（繼承 BleDriverSimComponentBase、
# 內含 RfDriverCore＋BleLinkCore＋BleHalSim）等決定要上 SIL 拓樸時再補。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/BleDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/BleHalSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Wireless_Ports Components_Wireless_BleDriver)
register_fprime_module()
