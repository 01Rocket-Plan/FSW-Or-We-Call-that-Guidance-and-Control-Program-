# PowerMonitor — voltage rail sampler + Sim (Base ID 0x6000 / Sim 0x6080)
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/PowerMonitor.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/PowerMonitor.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PowerRailCore.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PowerRailSim.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()

# UvState state machine (Base ID 0x6200 / Sim 0x6280) lives under this folder.
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/UvState")
