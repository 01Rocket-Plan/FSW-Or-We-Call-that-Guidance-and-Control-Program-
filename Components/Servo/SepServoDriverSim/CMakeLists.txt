# SepServoDriverSim — Sim shell for SepServoDriver (0x3180)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Power_NvFlags
  Components_Servo_SimCore
  Components_Servo_ServoTypes
  Components_Servo_ModelCores
)
register_fprime_module()
