# Controller — DS 姿控 W3（MCU 組）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Controller.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Controller.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ControllerCore.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
