# ServoDriverSim（伺服組子Bot1）— servoDriverSim 0x3080、gimbalServoDriverSim 0x3280
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriverSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Servo_SimCore)
register_fprime_module()
