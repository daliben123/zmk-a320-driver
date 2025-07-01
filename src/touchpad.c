#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/input.h>

#define DT_DRV_COMPAT zmk_touchpad

// 寄存器定义（与原始代码一致）
#define REG_MOTION      0x02
#define REG_DELTA_X     0x03
#define REG_DELTA_Y     0x04
#define BIT_MOTION_MOT  (1 << 7)

// 滑动检测参数
#define SWIPE_THRESHOLD 5      // 最小滑动距离
#define SWIPE_DIRECTION_RATIO 2 // 方向判断比例

// 设备配置结构体
struct touchpad_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec shutdown_gpio;
    uint32_t swipe_cooldown_ms;
    uint32_t swipe_release_delay_ms;
};

// 设备运行时数据
struct touchpad_data {
    struct gpio_callback motion_cb;
    struct k_work motion_work;
    int64_t last_swipe_time;
};

// 中断处理（Zephyr回调模式）
static void motion_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct touchpad_data *data = CONTAINER_OF(cb, struct touchpad_data, motion_cb);
    k_work_submit(&data->motion_work); // 提交工作队列
}

// 主处理逻辑（在工作队列中执行）
static void process_motion(struct k_work *work) {
    struct touchpad_data *data = CONTAINER_OF(work, struct touchpad_data, motion_work);
    const struct device *dev = device_get_binding(DT_INST_LABEL(0));
    const struct touchpad_config *cfg = dev->config;

    uint8_t motion;
    i2c_reg_read_byte_dt(&cfg->i2c, REG_MOTION, &motion);

    if (motion & BIT_MOTION_MOT) {
        int8_t x, y;
        i2c_reg_read_byte_dt(&cfg->i2c, REG_DELTA_X, (uint8_t *)&x);
        i2c_reg_read_byte_dt(&cfg->i2c, REG_DELTA_Y, (uint8_t *)&y);

        // 坐标转换（与原始代码一致）
        x = ((x < 127) ? x : (x - 256)) * -1;
        y = ((y < 127) ? y : (y - 256));

        // 滑动检测逻辑
        if (zmk_keymap_layer_active(ALT_LAYER)) { // 检测Alt层激活
            int64_t now = k_uptime_get();
            if (now - data->last_swipe_time > cfg->swipe_cooldown_ms) {
                zmk_keycode_t key = ZMK_KEY_NONE;
                int abs_x = ABS(x);
                int abs_y = ABS(y);
                
                // 滑动方向判断
                if (abs_x >= SWIPE_THRESHOLD || abs_y >= SWIPE_THRESHOLD) {
                    if (abs_x > abs_y * SWIPE_DIRECTION_RATIO) {
                        // 水平滑动
                        key = (x < 0) ? KEY_LEFT : KEY_RIGHT;
                    } else if (abs_y > abs_x * SWIPE_DIRECTION_RATIO) {
                        // 垂直滑动
                        key = (y < 0) ? KEY_UP : KEY_DOWN;
                    }
                }
                
                if (key != ZMK_KEY_NONE) {
                    zmk_input_keypress(key, true);  // 按下
                    k_msleep(cfg->swipe_release_delay_ms);
                    zmk_input_keypress(key, false); // 释放
                    data->last_swipe_time = now;
                }
            }
        } else {
            // 原始触摸回调（需扩展为ZMK输入子系统）
            zmk_input_mouse_move(x, y); // 转换为鼠标移动事件
        }
    }
}

// 设备初始化
static int touchpad_init(const struct device *dev) {
    const struct touchpad_config *cfg = dev->config;
    struct touchpad_data *data = dev->data;

    // GPIO初始化
    gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&cfg->shutdown_gpio, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT | GPIO_INT_EDGE_FALLING);

    // 复位序列（与原始代码一致）
    gpio_pin_set_dt(&cfg->reset_gpio, 0);
    k_msleep(100);
    gpio_pin_set_dt(&cfg->reset_gpio, 1);

    // 中断和工作队列配置
    gpio_init_callback(&data->motion_cb, motion_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback_dt(&cfg->motion_gpio, &data->motion_cb);
    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    k_work_init(&data->motion_work, process_motion);

    return 0;
}

// Zephyr设备定义
#define TOUCHPAD_INIT(n) \
    static struct touchpad_config config_##n = { \
        .i2c = I2C_DT_SPEC_INST_GET(n), \
        .motion_gpio = GPIO_DT_SPEC_INST_GET(n, motion_gpios), \
        .reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios), \
        .shutdown_gpio = GPIO_DT_SPEC_INST_GET(n, shutdown_gpios), \
        .swipe_cooldown_ms = DT_INST_PROP(n, swipe_cooldown_ms), \
        .swipe_release_delay_ms = DT_INST_PROP(n, swipe_release_delay_ms) \
    }; \
    static struct touchpad_data data_##n; \
    DEVICE_DT_INST_DEFINE(n, touchpad_init, NULL, &data_##n, &config_##n, \
                          POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TOUCHPAD_INIT)
