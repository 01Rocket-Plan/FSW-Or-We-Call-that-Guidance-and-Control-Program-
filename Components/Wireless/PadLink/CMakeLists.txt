# PadLink — 共用幀 codec／指令驗證／FPP 型別（Issue #24）。Core host-testable.
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/PadLink.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/PadLinkCodec.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PadCmdGate.cpp"
)
register_fprime_module()
