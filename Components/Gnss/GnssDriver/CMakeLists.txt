# GnssDriver（TEL0157 真驅動）— GNSS 組；base id 0x5000（conventions）／Sim +0x80
# 安裝位置：DS／US 皆可
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/GnssDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/GnssDriver.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/GnssCore.cpp"
)
register_fprime_module()
