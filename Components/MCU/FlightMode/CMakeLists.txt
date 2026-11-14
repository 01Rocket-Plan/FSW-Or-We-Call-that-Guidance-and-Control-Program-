# FlightMode — 飛行模式狀態機（邏輯照 SSS flight_mode.py）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/FlightMode.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/FlightMode.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FlightModeCore.cpp"
)
set(MOD_DEPS RocketTypes Components_Power_NvFlags Components_Wireless_UmbilicalLink)
register_fprime_module()
