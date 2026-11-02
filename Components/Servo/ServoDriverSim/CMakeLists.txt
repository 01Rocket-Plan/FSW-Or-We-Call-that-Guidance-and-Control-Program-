# 元件指定（Sim）：模擬 SG90 伺服 ServoDriver（DsMcu 尾翼 TAIL_1–4／UsMcu 雲台 PITCH/ROLL）；Base ID servoDriverSim 0x3080、gimbalServoDriverSim 0x3280（channel_count=2）；無硬體 I/O。
# ServoDriverSim（伺服組子Bot1）— servoDriverSim 0x3080、gimbalServoDriverSim 0x3280
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriverSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Servo_SimCore)
register_fprime_module()
