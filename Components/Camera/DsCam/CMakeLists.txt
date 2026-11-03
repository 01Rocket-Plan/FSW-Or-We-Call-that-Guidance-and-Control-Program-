# DsCam — ESP32-CAM (model ESP32_CAM, role DS_CAM)
# CamTypes come from Components_Camera_Ov3640Driver (must be built first).
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Esp32Cam.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32CamCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32CamSimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32Cam.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32CamSim.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Camera_Ov3640Driver
)
register_fprime_module()
