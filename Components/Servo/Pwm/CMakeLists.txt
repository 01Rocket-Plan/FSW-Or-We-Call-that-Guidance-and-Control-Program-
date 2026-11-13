# PwmOut 抽象（Backend A：ESP32 LEDC／host stub）— 伺服馬達驅動組
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/PwmOut.cpp"
)
# 真 F´：沒有 .fpp 也沒有 MOD_DEPS 時，F´ 不給任何 include 路徑（issue #33）
set(MOD_DEPS Fw_Types)
register_fprime_module()
