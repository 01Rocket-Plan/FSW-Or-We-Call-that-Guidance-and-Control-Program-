cmake_minimum_required(VERSION 3.16)
project(RocketPlan01 C CXX)
# F´ 由 lib/fprime（git submodule，版本待 MCU 組鎖定）提供
include("${CMAKE_CURRENT_LIST_DIR}/lib/fprime/cmake/FPrime.cmake")
include("${FPRIME_FRAMEWORK_PATH}/cmake/FPrime-Code.cmake")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/RocketTypes")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Components")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Deployments/DownStage")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Deployments/UpStage")
