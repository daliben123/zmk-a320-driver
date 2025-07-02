/* touchpad.h - ZMK触控板驱动头文件 */
#ifndef ZMK_DRIVER_TOUCHPAD_H
#define ZMK_DRIVER_TOUCHPAD_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/mouse.h>
#include <zephyr/input/keyboard.h>
#include <zephyr/sys/util.h>

/* 触控板寄存器定义 */
#define TOUCHPAD_REG_PID         0x00
#define TOUCHPAD_REG_REV         0x01
#define TOUCHPAD_REG_MOTION      0x02
#define TOUCHPAD_REG_DELTA_X     0x03
#define TOUCHPAD_REG_DELTA_Y     0x04
#define TOUCHPAD_REG_DELTA_XY_H  0x05
#define TOUCHPAD_REG_CONFIG      0x11
#define TOUCHPAD_REG_OBSERV      0x2E
#define TOUCHPAD_REG_MBURST      0x42

/* 触控状态位定义 */
#define TOUCHPAD_BIT_MOTION_MOT  (1 << 7)
#define TOUCHPAD_BIT_MOTION_OVF  (1 << 4)

/* 配置位定义 */
#define TOUCHPAD_BIT_CONFIG_HIRES (1 << 7)

/* 触控模式定义 */
enum touchpad_mode {
    TOUCHPAD_MODE_NORMAL = 0,
    TOUCHPAD_MODE_SCROLL,
    TOUCHPAD_MODE_BOOST,
    TOUCHPAD_MODE_SLOW
};

/* 驱动API函数 */
int touchpad_set_mode(const struct device *dev, enum touchpad_mode mode);
int touchpad_enable(const struct device *dev);
int touchpad_disable(const struct device *dev);

#endif /* ZMK_DRIVER_TOUCHPAD_H */
