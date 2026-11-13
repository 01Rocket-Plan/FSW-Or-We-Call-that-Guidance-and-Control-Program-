# UsCamDeployment — 上級相機獨立板（OV3640 備援分支；main 是 OV7670）。本版是 Linux SIL（F´ v4.4.1）；ESP32 版跟 MCU 組 ESP32 平台 PR 一起。
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Top/")

register_fprime_deployment(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/Main.cpp"
    DEPENDS
        ${FPRIME_CURRENT_MODULE}_Top
)
