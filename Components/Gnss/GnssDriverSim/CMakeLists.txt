# GnssDriverSim（TEL0157 NMEA 模擬）— GNSS 組（子Bot1＋組長整合）；base id base+0x80
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/GnssDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/GnssDriverSim.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/GnssSimDevice.cpp"
)
# 核心在 GnssDriver 模組；若單獨建置需一併連結 GnssCore.cpp
register_fprime_module()
