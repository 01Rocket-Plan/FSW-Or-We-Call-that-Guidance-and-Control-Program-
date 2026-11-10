# CamTypes — 相機共用 FPP 型別＋影像分塊埠（DS／US 相機、SdLogger 都依賴這個模組）
set(SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/CamTypes.fpp")
set(MOD_DEPS Fw_Types Fw_Buffer)
register_fprime_module()
