# ZMK触控板驱动CMake构建脚本

# 添加驱动源文件
target_sources(app PRIVATE
    src/touchpad.c
)

# 添加头文件路径
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 添加设备树绑定路径
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# 添加设备树覆盖文件路径
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/dts/overlays)
    set(DTC_OVERLAY_FILE ${DTC_OVERLAY_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/dts/overlays/${BOARD}.overlay)
endif()
