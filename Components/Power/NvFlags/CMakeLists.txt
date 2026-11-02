# NvFlags — NVS dual-slot flags (Base ID 0x6100 / Sim 0x6180)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/NvFlagsPorts.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/NvFlags.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/NvFlags.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NvFlagsCore.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
