# Ov3640Driver — US OV3640（SCCB + DVP）Core／Sim／FPP／F´ 外殼
# 型號 OV3640、角色 US_CAM；host 測試見 run_host_tests.sh
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/CamTypes.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640Driver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640Core.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640SimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640Driver.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640DriverSim.cpp"
)
register_fprime_module()
