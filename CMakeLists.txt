# ZMK触控板驱动CMake构建脚本

# 确保Zephyr构建系统正确识别驱动类型
if(NOT DEFINED ZEPHYR_DRIVER)
    set(ZEPHYR_DRIVER touchpad)
endif()

# 添加驱动源文件
target_sources(app PRIVATE
    src/touchpad.c
)

# 添加头文件路径
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 设备树配置部分
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# 设备树绑定文件路径（.dtsi）
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dts/touchpad.dtsi
)

# 设备树覆盖文件路径（.overlay）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/dts/overlays)
    set(DTC_OVERLAY_FILE ${DTC_OVERLAY_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/dts/overlays/${BOARD}.overlay)
    message(STATUS "Using overlay: ${CMAKE_CURRENT_SOURCE_DIR}/dts/overlays/${BOARD}.overlay")
endif()

# Kconfig配置选项
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/kconfig/touchpad.conf
)

# 驱动依赖的模块
if(DT_NODELABEL_EXISTS(trackpad))
    # 启用触摸板驱动的Kconfig选项
    set(KCONFIG_CONFIG "+CONFIG_TOUCHPAD=y")
    set(KCONFIG_CONFIG "+CONFIG_TOUCHPAD_I2C=y")
    
    # 依赖的Zephyr模块
    target_link_libraries(app PRIVATE
        zephyr:driver:i2c
        zephyr:driver:gpio
        zmk:events
        zmk:gestures
    )
    
    message(STATUS "Touchpad driver enabled for ${BOARD}")
else()
    message(STATUS "Touchpad driver not enabled, no device tree node found")
endif()

# 编译选项配置
target_compile_definitions(app PRIVATE
    TOUCHPAD_I2C_ADDR=${TOUCHPAD_I2C_ADDR}
)

# 导出驱动信息供其他模块使用
set(TOUCHPAD_DRIVER_SOURCES src/touchpad.c)
set(TOUCHPAD_DTS_BINDINGS ${CMAKE_CURRENT_SOURCE_DIR}/dts/touchpad.dtsi)
export(TOUCHPAD_DRIVER_SOURCES TOUCHPAD_DTS_BINDINGS)
