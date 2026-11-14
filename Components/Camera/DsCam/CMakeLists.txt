# DsCam — ESP32-CAM (model ESP32_CAM, role DS_CAM)
# CamTypes（FPP 自動碼）來自 Components_Camera_CamTypes。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Esp32Cam.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32CamCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32CamSimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32Cam.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Esp32CamSim.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Camera_CamTypes
)
register_fprime_module()

# 單元測試：真 F´ 型別＋gtest（原 host 測試搬進來，不再用手寫型別）
set(UT_SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/test/Esp32CamSimTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/test/Esp32CamWrapperLogicTest.cpp"
)
register_fprime_ut()
