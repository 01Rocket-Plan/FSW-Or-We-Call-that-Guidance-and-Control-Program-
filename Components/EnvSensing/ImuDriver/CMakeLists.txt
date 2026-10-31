set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ImuDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ImuDriver.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ImuDriverSim.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/imu_core.c"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
