# FlightMode — 狀態機殼（邏輯：Edward 本人）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/FlightMode.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/FlightMode.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FlightModeCore.cpp"
)
set(MOD_DEPS RocketTypes Components_Power_NvFlags)
register_fprime_module()
