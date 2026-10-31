set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/UsMpu6050.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/UsMpu6050.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UsMpu6050Sim.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mpu6050_core.c"
)
set(MOD_DEPS RocketTypes)
# ImuDriver provides imu_core.h; include path via parent when fprime lands
register_fprime_module()
