set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/ServoDriver.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
