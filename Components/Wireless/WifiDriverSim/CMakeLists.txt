# WifiDriverSim — WiFi 近距離備用傳輸 Sim，Base ID 0x8980（＝0x8900+0x80）。
# 埠集合與 WifiDriver／RfDriver 相同，不宣告硬體型號／角色。
# 與 BleDriverSim 相同做法：先有 FPP 元件＋可測的 WifiHalSim（IWifiHal 模擬）；F´ 外殼
# （繼承 WifiDriverSimComponentBase、內含 RfDriverCore＋WifiLinkCore＋WifiHalSim）等決定要上 SIL 拓樸時再補。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/WifiDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/WifiHalSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Wireless_Ports Components_Wireless_WifiDriver)
register_fprime_module()
