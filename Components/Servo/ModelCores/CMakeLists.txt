# ModelCores — Sg90Core / Mg996rCore + SimDevices (host-testable, no F´ deps)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Sg90Core.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mg996rCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Sg90SimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mg996rSimDevice.cpp"
)
register_fprime_module()
