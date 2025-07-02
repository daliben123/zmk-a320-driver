/* touchpad.h - ZMK触摸板驱动头文件 */

#ifndef ZMK_DRIVERS_TOUCHPAD_H_
#define ZMK_DRIVERS_TOUCHPAD_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/input.h>
#include <zmk/events/position_state_changed.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 寄存器定义 */
enum {
    TOUCHPAD_REG_PID        = 0x00,
    TOUCHPAD_REG_REV        = 0x01,
    TOUCHPAD_REG_MOTION     = 0x02,
    TOUCHPAD_REG_DELTA_X    = 0x03,
    TOUCHPAD_REG_DELTA_Y    = 0x04,
    TOUCHPAD_REG_DELTA_XY_H = 0x05,
    TOUCHPAD_REG_CONFIG     = 0x11,
    TOUCHPAD_REG_OBSERV     = 0x2E,
    TOUCHPAD_REG_MBURST     = 0x42,
};

/* 位定义 */
#define TOUCHPAD_BIT_MOTION_MOT (1 << 7)
#define TOUCHPAD_BIT_MOTION_OVF (1 << 4)

/* 模式定义 */
typedef enum {
    TOUCHPAD_MODE_MOUSE = 0,
    TOUCHPAD_MODE_SCROLL,
} touchpad_mode_t;

typedef enum {
    TOUCHPAD_SPEED_NORMAL = 0,
    TOUCHPAD_SPEED_BOOST,
    TOUCHPAD_SPEED_SLOW,
} touchpad_speed_t;

/* 设备配置结构体 */
struct touchpad_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec shutdown_gpio;
    uint32_t poll_interval_ms;
    uint8_t scroll_speed_divider;
    uint8_t boost_speed_multiplier;
    uint8_t slow_speed_divider;
};

/* 设备运行时数据 */
struct touchpad_data {
    struct gpio_callback motion_cb_data;
    struct k_work_delayable work;
    struct zmk_relative_position_state state;
    bool is_scroll_mode;
    bool is_boost_mode;
    bool is_slow_mode;
};

/* 驱动API结构体 */
struct touchpad_driver_api {
    int (*set_mode)(const struct device *dev, touchpad_mode_t mode);
    int (*set_speed)(const struct device *dev, touchpad_speed_t speed);
};

/* 函数原型声明 */
int touchpad_init(const struct device *dev);
void touchpad_work_handler(struct k_work *work);
int touchpad_set_scroll_mode(const struct device *dev, bool enable);
int touchpad_set_boost_mode(const struct device *dev, bool enable);
int touchpad_set_slow_mode(const struct device *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ZMK_DRIVERS_TOUCHPAD_H_ */
