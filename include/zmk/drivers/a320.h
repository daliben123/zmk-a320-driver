/* touchpad.h - ZMK触摸板驱动头文件 */

#ifndef ZMK_DRIVERS_TOUCHPAD_H_
#define ZMK_DRIVERS_TOUCHPAD_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/input.h>

/* 寄存器定义 */
#define TOUCHPAD_REG_MOTION      0x02
#define TOUCHPAD_REG_DELTA_X     0x03
#define TOUCHPAD_REG_DELTA_Y     0x04
#define TOUCHPAD_BIT_MOTION_MOT  (1 << 7)

/* 滑动检测参数配置 */
#define TOUCHPAD_SWIPE_THRESHOLD 5      /* 最小滑动距离 */
#define TOUCHPAD_SWIPE_DIRECTION_RATIO 2 /* 方向判断比例 */

/* 设备配置结构体 */
struct touchpad_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec shutdown_gpio;
    uint32_t swipe_cooldown_ms;
    uint32_t swipe_release_delay_ms;
};

/* 设备运行时数据 */
struct touchpad_data {
    struct gpio_callback motion_cb;
    struct k_work motion_work;
    int64_t last_swipe_time;
};

/* 函数原型声明 */
int touchpad_init(const struct device *dev);
void touchpad_process_motion(struct k_work *work);

#endif /* ZMK_DRIVERS_TOUCHPAD_H_ */
