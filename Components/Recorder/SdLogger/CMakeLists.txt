####
# SdLogger — dictionary (子Bot1) + DS/US Core/Sim implementation (子Bot2)
####
# 相機影像埠 CamImageOut 由相機組 Components/Camera/CamTypes 定義（issue #32：原本這裡又定義一次，
# 整個 repo 用真 F´ configure 失敗）。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SdLogger.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdCardCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdCardSimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdLoggerCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdLogger.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdLoggerSim.cpp"
)
set(MOD_DEPS
  RocketTypes
  Components_Camera_CamTypes
)
register_fprime_module()

set(UT_SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SdLogger.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/test/ut/SdLoggerDictIdTest.cpp"
)
set(UT_MOD_DEP_LIST)
register_fprime_ut()
