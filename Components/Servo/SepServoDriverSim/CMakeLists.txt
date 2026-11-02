# SepServoDriverSim（伺服組子Bot1）— 0x3180；NvFlags 埠型別來自 Power.NvFlagsPorts
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Power_NvFlags Components_Servo_SimCore)
register_fprime_module()
