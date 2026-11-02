# SepServoDriver — MG996R SEP_1–4 (formal model/role in ServoTypes; NvFlags via Power)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriver.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Servo_Pwm
  Components_Servo_ServoTypes
  Components_Power_NvFlags
  Components_Servo_ModelCores
)
register_fprime_module()
