# TlmSend — Base ID 0x8000。Core host-testable.
# F´ 外殼 TlmSend.hpp／.cpp（繼承 TlmSendComponentBase、呼叫 Core）；DsMcu 拓樸 rg100Hz（Issue #60）。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/TlmSend.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/TlmSendCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/TlmSend.cpp"
)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest；原 host 測試搬進來）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/TlmSendHostTest.cpp")
register_fprime_ut()
