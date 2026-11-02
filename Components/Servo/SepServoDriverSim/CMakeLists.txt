# SepServoDriverSim（伺服組子Bot1）— 0x3180；SepNvPlaceholder 型別沿用 SepServoDriver 模組
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Servo_SepServoDriver Components_Servo_SimCore)
register_fprime_module()
