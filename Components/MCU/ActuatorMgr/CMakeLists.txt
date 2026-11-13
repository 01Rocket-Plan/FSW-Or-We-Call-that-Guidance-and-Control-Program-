# ActuatorMgr — DS 作動管理 W4（MCU 組）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ActuatorMgr.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ActuatorMgr.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ActuatorMgrCore.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
