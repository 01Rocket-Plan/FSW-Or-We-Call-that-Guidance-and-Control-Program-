# SilImuFeed — SIL 佔位 IMU（MCU 組；ImuDriverSim 可用後刪除）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SilImuFeed.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SilImuFeed.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
