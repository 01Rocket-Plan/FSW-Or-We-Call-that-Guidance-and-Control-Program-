# Servo SimCore（伺服組子Bot1）：無框架核心，給 ServoDriverSim／SepServoDriverSim 共用
set(SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/ServoSimCore.cpp")
set(MOD_DEPS Fw_Types)
register_fprime_module()
