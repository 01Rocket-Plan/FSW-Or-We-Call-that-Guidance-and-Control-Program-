# Servo PWM HAL（ESP32 LEDC／host stub）— 伺服馬達驅動組
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ServoPwmHal.cpp"
)
register_fprime_module()
