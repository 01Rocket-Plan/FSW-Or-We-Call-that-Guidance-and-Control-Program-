# ComQueue — Base ID 0x8200。Core host-testable.
# F´ 外殼 ComQueue.hpp／.cpp（繼承 ComQueueComponentBase、呼叫 Core）；DsMcu 拓樸 rg100Hz（Issue #60）。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ComQueue.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ComQueueCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ComQueue.cpp"
)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest；原 host 測試搬進來）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/ComQueueHostTest.cpp")
register_fprime_ut()
