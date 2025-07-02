// drivers/input/trackpad.c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_REGISTER(trackpad, CONFIG_ZMK_TOUCHPAD_LOG_LEVEL);

// 驱动私有数据结构
struct trackpad_data {
    struct gpio_callback motion_cb_data;
    struct k_work_delayable work;
    struct zmk_relative_position_state state;
    bool is_scroll_mode;
    bool is_boost_mode;
    bool is_slow_mode;
};

// 驱动配置结构
struct trackpad_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec shutdown_gpio;
    uint32_t poll_interval_ms;
    uint8_t scroll_speed_divider;
    uint8_t boost_speed_multiplier;
    uint8_t slow_speed_divider;
};

// 寄存器定义
enum {
    REG_PID        = 0x00,
    REG_REV        = 0x01,
    REG_MOTION     = 0x02,
    REG_DELTA_X    = 0x03,
    REG_DELTA_Y    = 0x04,
    REG_DELTA_XY_H = 0x05,
    REG_CONFIG     = 0x11,
    REG_OBSERV     = 0x2E,
    REG_MBURST     = 0x42,
};

// 位定义
#define BIT_MOTION_MOT (1 << 7)
#define BIT_MOTION_OVF (1 << 4)

// 从指定寄存器读取8位数据
static int trackpad_read_register(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct trackpad_config *cfg = dev->config;
    return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

// 辅助函数：将原始触摸板数据转换为鼠标移动值
static int16_t convert_to_mouse_delta(int8_t raw, float scale_factor) {
    // 转换为有符号值
    int16_t signed_val = (int16_t)raw;
    
    // 应用缩放因子
    float scaled_val = signed_val * scale_factor;
    
    // 限制在合理范围内
    if (scaled_val > INT16_MAX) return INT16_MAX;
    if (scaled_val < INT16_MIN) return INT16_MIN;
    
    return (int16_t)scaled_val;
}

// 处理触摸板数据的工作函数
static void trackpad_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct device *dev = CONTAINER_OF(dwork, struct trackpad_data, work);
    const struct trackpad_config *cfg = dev->config;
    struct trackpad_data *data = dev->data;
    uint8_t motion_status;
    int8_t x, y;
    int err;

    // 读取运动状态
    err = trackpad_read_register(dev, REG_MOTION, &motion_status);
    if (err) {
        LOG_ERR("Failed to read motion register: %d", err);
        goto reschedule;
    }

    if (motion_status & BIT_MOTION_MOT) {
        // 读取X和Y方向的移动数据
        err = trackpad_read_register(dev, REG_DELTA_X, (uint8_t *)&x);
        if (err) {
            LOG_ERR("Failed to read delta X: %d", err);
            goto reschedule;
        }

        err = trackpad_read_register(dev, REG_DELTA_Y, (uint8_t *)&y);
        if (err) {
            LOG_ERR("Failed to read delta Y: %d", err);
            goto reschedule;
        }

        // 根据当前模式处理移动数据
        if (data->is_scroll_mode) {
            // 滚动模式：将移动转换为水平和垂直滚动
            data->state.h = convert_to_mouse_delta(x, 1.0f / cfg->scroll_speed_divider);
            data->state.v = convert_to_mouse_delta(y, -1.0f / cfg->scroll_speed_divider);
            data->state.x = 0;
            data->state.y = 0;
        } else {
            // 鼠标模式：根据速度模式调整移动速度
            float speed_factor = -1.5f; // 默认速度因子
            
            if (data->is_boost_mode)
                speed_factor *= cfg->boost_speed_multiplier;
            else if (data->is_slow_mode)
                speed_factor /= cfg->slow_speed_divider;
                
            // 应用速度因子并设置鼠标报告
            data->state.x = convert_to_mouse_delta(x, speed_factor);
            data->state.y = convert_to_mouse_delta(y, -speed_factor); // Y轴方向修正
            data->state.h = 0;
            data->state.v = 0;
        }

        // 发布相对位置变化事件
        struct zmk_relative_position_state_changed evt = {
            .state = data->state,
            .source = ZMK_REL_POS_SRC_TOUCHPAD,
        };
        ZMK_EVENT_RAISE(evt);
    }

reschedule:
    // 安排下一次轮询
    k_work_reschedule(dwork, K_MSEC(cfg->poll_interval_ms));
}

// 运动检测引脚中断回调
static void trackpad_motion_callback(const struct device *dev, struct gpio_callback *cb,
                                    uint32_t pins) {
    struct trackpad_data *data = CONTAINER_OF(cb, struct trackpad_data, motion_cb_data);
    // 立即安排处理工作
    k_work_reschedule(&data->work, K_NO_WAIT);
}

// 设置模式的API函数
int trackpad_set_scroll_mode(const struct device *dev, bool enable) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    struct trackpad_data *data = dev->data;
    data->is_scroll_mode = enable;
    return 0;
}

int trackpad_set_boost_mode(const struct device *dev, bool enable) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    struct trackpad_data *data = dev->data;
    data->is_boost_mode = enable;
    return 0;
}

int trackpad_set_slow_mode(const struct device *dev, bool enable) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    struct trackpad_data *data = dev->data;
    data->is_slow_mode = enable;
    return 0;
}

// 驱动初始化函数
static int trackpad_init(const struct device *dev) {
    const struct trackpad_config *cfg = dev->config;
    struct trackpad_data *data = dev->data;
    uint8_t pid, rev;
    int err;

    // 检查I2C设备是否就绪
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready", cfg->i2c.bus->name);
        return -ENODEV;
    }

    // 配置复位引脚
    if (!device_is_ready(cfg->reset_gpio.port)) {
        LOG_ERR("Reset GPIO device %s is not ready", cfg->reset_gpio.port->name);
        return -ENODEV;
    }
    
    err = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure reset GPIO: %d", err);
        return err;
    }

    // 配置关机引脚
    if (!device_is_ready(cfg->shutdown_gpio.port)) {
        LOG_ERR("Shutdown GPIO device %s is not ready", cfg->shutdown_gpio.port->name);
        return -ENODEV;
    }
    
    err = gpio_pin_configure_dt(&cfg->shutdown_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure shutdown GPIO: %d", err);
        return err;
    }

    // 配置运动检测引脚
    if (!device_is_ready(cfg->motion_gpio.port)) {
        LOG_ERR("Motion GPIO device %s is not ready", cfg->motion_gpio.port->name);
        return -ENODEV;
    }
    
    err = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (err) {
        LOG_ERR("Failed to configure motion GPIO: %d", err);
        return err;
    }

    // 硬件复位序列
    LOG_INF("Resetting trackpad...");
    gpio_pin_set_dt(&cfg->shutdown_gpio, 0);
    k_msleep(10);
    gpio_pin_set_dt(&cfg->reset_gpio, 0);
    k_msleep(100);
    gpio_pin_set_dt(&cfg->reset_gpio, 1);
    k_msleep(100);
    gpio_pin_set_dt(&cfg->shutdown_gpio, 1);
    k_msleep(100);

    // 读取设备ID（可选，用于调试）
    err = trackpad_read_register(dev, REG_PID, &pid);
    if (err) {
        LOG_WRN("Failed to read product ID: %d", err);
    } else {
        err = trackpad_read_register(dev, REG_REV, &rev);
        if (err) {
            LOG_WRN("Failed to read revision ID: %d", err);
        } else {
            LOG_INF("Trackpad detected - PID: 0x%02X, REV: 0x%02X", pid, rev);
        }
    }

    // 设置运动检测中断
    err = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data,
                          trackpad_motion_callback);
    if (err) {
        LOG_ERR("Failed to add motion callback: %d", err);
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (err) {
        LOG_ERR("Failed to configure motion interrupt: %d", err);
        return err;
    }

    // 初始化工作队列
    k_work_init_delayable(&data->work, trackpad_work_handler);
    
    // 安排首次轮询
    k_work_reschedule(&data->work, K_MSEC(cfg->poll_interval_ms));

    LOG_INF("Trackpad driver initialized");
    return 0;
}

// 驱动API结构
static const struct trackpad_driver_api trackpad_api = {
    .set_scroll_mode = trackpad_set_scroll_mode,
    .set_boost_mode = trackpad_set_boost_mode,
    .set_slow_mode = trackpad_set_slow_mode,
};

// 设备树绑定信息
static const struct trackpad_config trackpad_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .motion_gpio = GPIO_DT_SPEC_INST_GET(0, motion_gpios),
    .reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
    .shutdown_gpio = GPIO_DT_SPEC_INST_GET(0, shutdown_gpios),
    .poll_interval_ms = DT_INST_PROP(0, poll_interval_ms),
    .scroll_speed_divider = DT_INST_PROP(0, scroll_speed_divider),
    .boost_speed_multiplier = DT_INST_PROP(0, boost_speed_multiplier),
    .slow_speed_divider = DT_INST_PROP(0, slow_speed_divider),
};

// 驱动私有数据实例
static struct trackpad_data trackpad_data;

// 注册驱动
DEVICE_DT_INST_DEFINE(0, trackpad_init, NULL, &trackpad_data, &trackpad_config,
                     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &trackpad_api);
