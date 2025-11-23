#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

struct a320_data {
    struct gpio_callback motion_cb;
    struct k_work work;
    const struct device *dev;
    int16_t x_delta;
    int16_t y_delta;
};

struct a320_config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec reset_gpio;    // 始终包含GPIO成员
    struct gpio_dt_spec motion_gpio;   // 不再使用条件编译
    struct gpio_dt_spec shutdown_gpio; // 使用GPIO_DT_SPEC_INST_GET_OR初始化
};

// A320寄存器定义
#define Product_ID     0x00
#define Revision_ID    0x01
#define Motion         0x02
#define Delta_X        0x03
#define Delta_Y        0x04
#define Configuration_Bits 0x11
#define Observation    0x2E
#define Soft_RESET     0x3a

// 运动检测标志
#define BIT_MOTION_MOT (1 << 7)
#define BIT_MOTION_OVF (1 << 4)
