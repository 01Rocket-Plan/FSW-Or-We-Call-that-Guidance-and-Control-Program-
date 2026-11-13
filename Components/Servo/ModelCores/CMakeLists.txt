# ModelCores — Sg90Core / Mg996rCore + SimDevices (host-testable, no F´ deps)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Sg90Core.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mg996rCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Sg90SimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mg996rSimDevice.cpp"
)
# 真 F´：沒有 .fpp 也沒有 MOD_DEPS 時，F´ 不給任何 include 路徑（issue #33）
set(MOD_DEPS Fw_Types)
register_fprime_module()
