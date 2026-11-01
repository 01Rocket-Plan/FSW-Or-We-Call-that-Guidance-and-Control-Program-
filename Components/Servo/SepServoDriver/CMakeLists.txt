set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SepNvPlaceholder.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SepServoDriver.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
