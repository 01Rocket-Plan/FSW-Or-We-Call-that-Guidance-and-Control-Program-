# EnvSensingBus — 環境感測組共用的匯流排埠型別（RegWrite／RegRead／SpiReg*／FaultInject）
# issue #34：EnvSensingBus.fpp 原本沒進任何模組，用真 F´ 時 ImuDriver／UsMpu6050 找不到符號。
set(SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/EnvSensingBus.fpp")
set(MOD_DEPS Fw_Types Fw_Buffer)
register_fprime_module(Components_EnvSensing_Bus)
