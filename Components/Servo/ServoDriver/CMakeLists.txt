# 元件指定：型號 SG90 微型伺服。
# 用途（共用同一元件定義、拓樸中為兩個獨立實例）：
#  (a) DsMcu 尾翼 TAIL_1–4：Base ID 0x3000，channel_count=4，GPIO 25/26/27/14，LEDC 高速 ch0–3、timer0。
#  (b) UsMcu 相機二軸雲台 PITCH/ROLL：實例 gimbalServoDriver，Base ID 0x3200，channel_count=2，GPIO 25(PITCH)/26(ROLL)，LEDC 高速 ch0–1。
# 訊號：50 Hz PWM，脈寬參數夾限 1000–2000 µs，中立 1500 µs。
# 腳位為 MCU 組 v0.1 草案，待 Anatoly 確認，實際值以 Components/Servo/Pwm/ServoPwmPins.hpp 為準。
# 供電：供電依電力組定案（待確認）。
#
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriver.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Servo_Pwm
)
register_fprime_module()
