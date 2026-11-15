# PadController — Base ID 0x8700（PadMcu）。Core host-testable.
# F´ 外殼 PadController.hpp／.cpp（繼承 PadControllerComponentBase、呼叫 Core）；驗證器未接 → deny-by-default。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/PadController.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/PadControllerCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PadController.cpp"
)
set(MOD_DEPS Components_Wireless_PadLink)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest；原 host 測試搬進來）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/../PadLink/test/PadLinkHostTest.cpp")
set(UT_MOD_DEPS Components_Wireless_UmbilicalLink)
register_fprime_ut()
