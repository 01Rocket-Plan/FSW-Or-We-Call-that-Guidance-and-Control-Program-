# Wireless 共用埠（RfHealth…）。原本 WirelessPorts.fpp 沒進任何模組，真 F´ 找不到（同 #34）。
set(SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/WirelessPorts.fpp")
set(MOD_DEPS RocketTypes)
register_fprime_module()
