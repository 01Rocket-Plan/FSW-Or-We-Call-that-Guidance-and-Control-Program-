# 元件指定：型號 MG996R 標準伺服 ×4，級間分離 SEP_1–4，僅 DsMcu。
# Base ID 0x3100，GPIO 32/33/13/4，LEDC 高速 ch4–7、timer0。
# lock_us 1000 / fire_us 2000；fire_hold_s 0.5 後保持 fire_us。
# 每通道一生只釋放一次（經 Power.SepReleasedSet/Get NvFlags）。
# 腳位為 MCU 組 v0.1 草案，待 Anatoly 確認，實際值以 Components/Servo/Pwm/ServoPwmPins.hpp 為準。
# 供電：供電依電力組定案（待確認）。
#
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriver.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Servo_Pwm
  Components_Power_NvFlags
)
register_fprime_module()
