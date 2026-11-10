# UplinkAuth — Base ID 0x8300。HMAC-SHA256＋防重放（UplinkAuthCore）＋白名單閘（whitelist/）。
# TODO(無線組)：UplinkAuth.fpp＋F´ 外殼（指令進來 → HMAC → 白名單 → 轉發）— PR #31 合併後補（issue #37-3）。
# 目前沒有 .fpp：先當純 C++ 函式庫模組用真 F´ 建（白名單吃 RocketTypes.FlightModeId 自動碼）。
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/HmacSha256.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UplinkAuthCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/whitelist/WhitelistGate.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/whitelist/WhitelistSimTable.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()

# 單元測試（真 F´ UT＋gtest；原 host 測試搬進來）
set(UT_SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/test/UplinkAuthHostTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/whitelist/test/WhitelistHostTest.cpp"
)
register_fprime_ut()
