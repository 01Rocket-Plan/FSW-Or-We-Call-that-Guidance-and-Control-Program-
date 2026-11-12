####
# SdLogger — dictionary (子Bot1) + DS/US Core/Sim implementation (子Bot2)
####
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SdLogger.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdCardCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdCardSimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdLoggerCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdLogger.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SdLoggerSim.cpp"
)
set(MOD_DEPS
  Components_Camera_Ov3640Driver
  Components_Camera_DsCam
)
register_fprime_module()

set(UT_SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SdLogger.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/test/ut/SdLoggerDictIdTest.cpp"
)
set(UT_MOD_DEP_LIST)
register_fprime_ut()
