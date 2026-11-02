# 元件指定（Sim）：模擬 MG996R 級間分離伺服 SepServoDriver（DsMcu SEP_1–4）；Base ID 0x3180；無硬體 I/O。
# SepServoDriverSim（伺服組子Bot1）— 0x3180；NvFlags 埠型別來自 Power.NvFlagsPorts
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriverSim.cpp"
)
set(MOD_DEPS RocketTypes Components_Power_NvFlags Components_Servo_SimCore)
register_fprime_module()
