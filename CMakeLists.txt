# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

# Touchpad驱动源文件
zephyr_library_sources_ifdef(CONFIG_TOUCHPAD_DRIVER src/touchpad.c)

# 公共头文件目录
zephyr_include_directories(include)

# 应用程序自定义头文件目录
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

# Touchpad驱动头文件目录
zephyr_include_directories(include/drivers/input)
