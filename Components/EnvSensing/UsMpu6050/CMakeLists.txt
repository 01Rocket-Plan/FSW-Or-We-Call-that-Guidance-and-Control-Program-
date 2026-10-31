set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/UsMpu6050.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/UsMpu6050.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UsMpu6050Sim.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mpu6050_core.c"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
