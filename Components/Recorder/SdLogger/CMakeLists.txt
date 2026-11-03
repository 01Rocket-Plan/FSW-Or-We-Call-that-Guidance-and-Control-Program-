####
# SdLogger — dictionary FPP owned by 紀錄組子Bot1; implementation by 子Bot2.
####
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SdLogger.fpp"
)
# Implementation (SdLogger.cpp / media / queue) owned by 紀錄組子Bot2 — FPP dictionary only here.
register_fprime_module()

set(UT_SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/test/ut/SdLoggerDictIdTest.cpp"
)
register_fprime_ut()
