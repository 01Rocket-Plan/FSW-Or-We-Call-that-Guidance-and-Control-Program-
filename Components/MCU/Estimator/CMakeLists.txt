# Estimator — DS 估態 W1（MCU 組）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Estimator.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Estimator.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/EstimatorCore.cpp"
)
set(MOD_DEPS RocketTypes)
register_fprime_module()
