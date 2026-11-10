# Framer — Base ID 0x8100。Core host-testable.
# TODO(無線組)：F´ 外殼 Framer.hpp／.cpp（繼承 FramerComponentBase、呼叫 Core）— PR #31 合併後補（issue #37-3）。
# 外殼還沒寫之前，模組照樣用真 F´ 建：FPP 自動碼＋Core 都會編譯，只是還不能放進拓樸。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Framer.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/FramerCore.cpp"
)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest；原 host 測試搬進來）
set(UT_SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/test/FramerHostTest.cpp")
register_fprime_ut()
