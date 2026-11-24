# Framer — Base ID 0x8100。Core host-testable.
# F´ 外殼 Framer.hpp／.cpp（繼承 FramerComponentBase、呼叫 Core）；DsMcu 拓樸（Issue #60）。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Framer.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/FramerCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Framer.cpp"
)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest；原 host 測試搬進來）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/FramerHostTest.cpp")
register_fprime_ut()
