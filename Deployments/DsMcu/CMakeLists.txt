# DsMcuDeployment — DS 飛控 MCU。本版是 Linux SIL（F´ v4.4.1）；ESP32 版下一個 PR。
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Top/")

register_fprime_deployment(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/Main.cpp"
    DEPENDS
        ${FPRIME_CURRENT_MODULE}_Top
)
