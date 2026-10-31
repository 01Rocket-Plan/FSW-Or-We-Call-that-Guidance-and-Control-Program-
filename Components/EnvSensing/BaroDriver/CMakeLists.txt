# BaroDriver（DS BME680）與 BaroDriverSim — 環境感測組（子Bot2）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/BaroDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Bme680Core.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Bme680SimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/BaroDriver.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/BaroDriverSim.cpp"
)
register_fprime_module()
