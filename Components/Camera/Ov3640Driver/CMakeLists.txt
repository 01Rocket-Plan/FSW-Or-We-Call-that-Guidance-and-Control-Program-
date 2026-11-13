# Ov3640Driver — US OV3640（SCCB + DVP）Core／Sim／FPP／F´ 外殼【備援，不合 main】
# 型號 OV3640、角色 US_CAM；型別全部來自 FPP 自動碼（Components_Camera_CamTypes）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640Driver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640Core.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640SimDevice.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640Driver.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Ov3640DriverSim.cpp"
)
set(MOD_DEPS
  Components_Camera_CamTypes
)
register_fprime_module()

# 單元測試：真 F´ 型別＋gtest（取代原本手寫假自動碼 host_ac/ 的測試）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/Ov3640SimTest.cpp")
register_fprime_ut()
