# ServoDriverSim — Sim shell for ServoDriver (servoDriverSim 0x3080 / gimbal 0x3280)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriverSim.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Servo_SimCore
  Components_Servo_ServoTypes
)
register_fprime_module()
