# UvStateComp — under-voltage / shed request (Base ID 0x6200)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/UvState.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/UvStateComp.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Core/UvStateCore.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
