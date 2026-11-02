# ServoDriver — SG90 TAIL/GIMBAL PWM driver (formal model/role in ServoTypes)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriver.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Servo_Pwm
  Components_Servo_ServoTypes
)
register_fprime_module()
