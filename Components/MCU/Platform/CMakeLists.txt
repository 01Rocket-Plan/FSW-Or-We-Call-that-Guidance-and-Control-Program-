# Platform — 踢狗、迴圈監測、重開機原因（MCU 組）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Platform.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Platform.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PlatformCore.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()

# SIL 專用：墊上靜置 IMU 佔位（等 EnvSensing 的 ImuDriverSim 能用真框架編譯後移除）
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/SilImuFeed")
