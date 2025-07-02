/* touchpad.c - ZMK触控板驱动实现 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(touchpad, CONFIG_ZMK_DRIVER_TOUCHPAD_LOG_LEVEL);

#include "touchpad.h"

/* 设备私有数据结构 */
struct touchpad_data {
    struct gpio_callback motion_cb;
    struct k_work_delayable work;
    enum touchpad_mode current_mode;
    bool enabled;
    uint32_t last_swipe_time;
};

/* 设备配置结构 */
struct touchpad_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec shutdown_gpio;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec reset_gpio;
};

/* 私有函数声明 */
static int touchpad_read_reg(const struct device *dev, uint8_t reg, uint8_t *val);
static void touchpad_process_motion(const struct device *dev);
static void touchpad_motion_handler(const struct device *dev, int8_t x, int8_t y);
static void touchpad_gesture_handler(const struct device *dev, int8_t x, int8_t y);
static void touchpad_work_handler(struct k_work *work);

/* 读取寄存器值 */
static int touchpad_read_reg(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct touchpad_config *cfg = dev->config;
    uint8_t buf[1] = {reg};
    
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    int ret = i2c_write_read_dt(&cfg->i2c, buf, 1, val, 1);
    if (ret < 0) {
        LOG_WRN("Failed to read register 0x%02X: %d", reg, ret);
    }
    return ret;
}

/* 处理触控事件工作项 */
static void touchpad_work_handler(struct k_work *work) {
    struct touchpad_data *data = CONTAINER_OF(work, struct touchpad_data, work);
    const struct device *dev = CONTAINER_OF(data, const struct device, data);
    
    touchpad_process_motion(dev);
}

/* 处理触控事件 */
static void touchpad_process_motion(const struct device *dev) {
    const struct touchpad_config *cfg = dev->config;
    struct touchpad_data *data = dev->data;
    
    uint8_t motion;
    if (touchpad_read_reg(dev, TOUCHPAD_REG_MOTION, &motion) < 0) {
        return;
    }
    
    if (!(motion & TOUCHPAD_BIT_MOTION_MOT)) {
        return;
    }
    
    int8_t x, y;
    if (touchpad_read_reg(dev, TOUCHPAD_REG_DELTA_X, (uint8_t *)&x) < 0 ||
        touchpad_read_reg(dev, TOUCHPAD_REG_DELTA_Y, (uint8_t *)&y) < 0) {
        return;
    }
    
    /* 处理补码值 */
    x = (x < 127) ? x : (x - 256);
    y = (y < 127) ? y : (y - 256);
    
    /* 处理触控数据 */
    touchpad_motion_handler(dev, x, y);
}

/* 处理鼠标移动和滚动 */
static void touchpad_motion_handler(const struct device *dev, int8_t x, int8_t y) {
    struct touchpad_data *data = dev->data;
    struct input_event ev = {0};
    struct mouse_move move = {0};
    
    switch (data->current_mode) {
        case TOUCHPAD_MODE_SCROLL:
            move.h = x / 6;  // 滚动速度
            move.v = -y / 6;
            ev.type = INPUT_EVENT_MOUSE;
            ev.mouse = move;
            break;
            
        case TOUCHPAD_MODE_BOOST:
            x = x * -3;  // 加速模式 (1.5 * 2)
            y = y * 3;
            move.x = x;
            move.y = y;
            ev.type = INPUT_EVENT_MOUSE;
            ev.mouse = move;
            break;
            
        case TOUCHPAD_MODE_SLOW:
            x = x * -0.75;  // 减速模式 (1.5 / 2)
            y = y * 0.75;
            move.x = x;
            move.y = y;
            ev.type = INPUT_EVENT_MOUSE;
            ev.mouse = move;
            break;
            
        case TOUCHPAD_MODE_NORMAL:
        default:
            x = x * -1.5;  // 正常模式
            y = y * 1.5;
            move.x = x;
            move.y = y;
            ev.type = INPUT_EVENT_MOUSE;
            ev.mouse = move;
            break;
    }
    
    input_submit(&ev);
}

/* 处理手势识别 */
static void touchpad_gesture_handler(const struct device *dev, int8_t x, int8_t y) {
    struct touchpad_data *data = dev->data;
    uint32_t current_time = k_uptime_get();
    
    /* 滑动冷却时间检查 */
    if (current_time - data->last_swipe_time < 100) {
        return;
    }
    
    char key = 0;
    if (ABS(y) > 15 && ABS(x) < 5) {
        key = (y < 0) ? KEY_UP : KEY_DOWN;
    } else if (ABS(x) > 15 && ABS(y) < 5) {
        key = (x < 0) ? KEY_LEFT : KEY_RIGHT;
    }
    
    if (key) {
        /* 发送按键事件 */
        struct input_event ev = {
            .type = INPUT_EVENT_KEY,
            .key = key,
            .state = INPUT_KEY_STATE_PRESSED
        };
        input_submit(&ev);
        
        /* 安排按键释放 */
        k_work_reschedule(&data->work, K_MSEC(10));
        
        data->last_swipe_time = current_time;
    }
}

/* GPIO中断处理函数 */
static void touchpad_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct touchpad_data *data = CONTAINER_OF(cb, struct touchpad_data, motion_cb);
    const struct device *tpad_dev = CONTAINER_OF(data, const struct device, data);
    
    k_work_submit(&data->work);
}

/* 驱动初始化函数 */
static int touchpad_init(const struct device *dev) {
    const struct touchpad_config *cfg = dev->config;
    struct touchpad_data *data = dev->data;
    
    LOG_INF("Initializing touchpad driver");
    
    /* 检查I2C设备 */
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }
    
    /* 配置复位引脚 */
    if (device_is_ready(cfg->reset_gpio.port)) {
        int ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset pin: %d", ret);
            return ret;
        }
        
        /* 复位序列 */
        gpio_pin_set_dt(&cfg->reset_gpio, 0);
        k_msleep(100);
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_msleep(100);
    }
    
    /* 配置休眠引脚 */
    if (device_is_ready(cfg->shutdown_gpio.port)) {
        int ret = gpio_pin_configure_dt(&cfg->shutdown_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure shutdown pin: %d", ret);
            return ret;
        }
    }
    
    /* 配置中断引脚 */
    if (device_is_ready(cfg->motion_gpio.port)) {
        int ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure motion pin: %d", ret);
            return ret;
        }
        
        ret = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb, 
                               touchpad_irq_handler);
        if (ret < 0) {
            LOG_ERR("Failed to add GPIO callback: %d", ret);
            return ret;
        }
        
        ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure interrupt: %d", ret);
            return ret;
        }
    }
    
    /* 初始化工作项 */
    k_work_init_delayable(&data->work, touchpad_work_handler);
    
    /* 初始化设备状态 */
    data->current_mode = TOUCHPAD_MODE_NORMAL;
    data->enabled = true;
    data->last_swipe_time = 0;
    
    LOG_INF("Touchpad driver initialized successfully");
    return 0;
}

/* 驱动API实现 */
int touchpad_set_mode(const struct device *dev, enum touchpad_mode mode) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    struct touchpad_data *data = dev->data;
    data->current_mode = mode;
    LOG_DBG("Touchpad mode set to %d", mode);
    return 0;
}

int touchpad_enable(const struct device *dev) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    struct touchpad_data *data = dev->data;
    data->enabled = true;
    LOG_DBG("Touchpad enabled");
    return 0;
}

int touchpad_disable(const struct device *dev) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    struct touchpad_data *data = dev->data;
    data->enabled = false;
    LOG_DBG("Touchpad disabled");
    return 0;
}

/* 驱动操作表 */
static const struct touchpad_driver_api touchpad_api = {
    .set_mode = touchpad_set_mode,
    .enable = touchpad_enable,
    .disable = touchpad_disable
};

/* 设备定义 */
#define TOUCHPAD_DEFINE(inst)                                                          \
    static struct touchpad_data touchpad_data_##inst;                                  \
                                                                                         \
    static const struct touchpad_config touchpad_config_##inst = {                    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                              \
        .shutdown_gpio = GPIO_DT_SPEC_INST_GET(inst, shutdown_gpios),                 \
        .motion_gpio = GPIO_DT_SPEC_INST_GET(inst, motion_gpios),                     \
        .reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                       \
    };                                                                                   \
                                                                                         \
    DEVICE_DT_INST_DEFINE(inst,                                                         \
                         touchpad_init,                                                 \
                         NULL,                                                          \
                         &touchpad_data_##inst,                                         \
                         &touchpad_config_##inst,                                       \
                         POST_KERNEL,                                                   \
                         CONFIG_APPLICATION_INIT_PRIORITY,                              \
                         &touchpad_api);

/* 为每个设备树实例创建驱动 */
DT_INST_FOREACH_STATUS_OKAY(TOUCHPAD_DEFINE)
