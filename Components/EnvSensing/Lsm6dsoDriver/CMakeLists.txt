# Lsm6dsoDriver — DS IMU 備援料 ST LSM6DSO（I2C）Core／Sim／FPP／F´ 外殼【備援分支，不合 main】
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Lsm6dsoDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Lsm6dsoDriver.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Lsm6dsoDriverSim.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/lsm6dso_core.c"
)
set(MOD_DEPS RocketTypes Components_EnvSensing_Bus)
register_fprime_module()
