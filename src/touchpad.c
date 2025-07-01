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

// 上电序列函数
static void touchpad_power_up_sequence(const struct touchpad_config *cfg) {
    // 步骤2: 设置引脚（假设Shutdown和IO_Select是GPIO引脚）
    gpio_pin_set_dt(&cfg->shutdown_gpio, 0);
    // 这里假设使用TWI，IO_Select置低
    // gpio_pin_set_dt(&io_select_gpio, 0); 

    // 步骤3: 设置TWI从地址（假设A0和A1已设置）

    // 步骤4: 复位NRST引脚
    gpio_pin_set_dt(&cfg->reset_gpio, 0);
    k_msleep(100);
    gpio_pin_set_dt(&cfg->reset_gpio, 1);

    // 步骤7: 向地址0x60写入0xE4
    uint8_t value = 0xE4;
    i2c_reg_write_byte_dt(&cfg->i2c, 0x60, value);

    // 步骤8: 设置速度切换
    value = 0x12;
    i2c_reg_write_byte_dt(&cfg->i2c, 0x62, value);
    value = 0x0E;
    i2c_reg_write_byte_dt(&cfg->i2c, 0x63, value);

    // 步骤9: 检查寄存器0x64 - 0x6b
    uint8_t expected_values_64_6b[] = {0x08, 0x06, 0x40, 0x08, 0x48, 0x0a, 0x50, 0x48};
    for (int i = 0; i < 8; i++) {
        uint8_t reg_value;
        i2c_reg_read_byte_dt(&cfg->i2c, 0x64 + i, &reg_value);
        if (reg_value != expected_values_64_6b[i]) {
            // 处理错误
            printk("Error: Register 0x%x value mismatch, expected 0x%x, got 0x%x\n", 0x64 + i, expected_values_64_6b[i], reg_value);
        }
    }

    // 步骤10: 检查Assert/De-assert寄存器
    uint8_t expected_values_6d_71[] = {0xc4, 0x34, 0x3c, 0x18, 0x20};
    for (int i = 0; i < 5; i++) {
        uint8_t reg_value;
        i2c_reg_read_byte_dt(&cfg->i2c, 0x6d + i, &reg_value);
        if (reg_value != expected_values_6d_71[i]) {
            // 处理错误
            printk("Error: Register 0x%x value mismatch, expected 0x%x, got 0x%x\n", 0x6d + i, expected_values_6d_71[i], reg_value);
        }
    }

    // 步骤11: 检查手指存在检测寄存器
    uint8_t expected_value_75 = 0x50;
    uint8_t reg_value_75;
    i2c_reg_read_byte_dt(&cfg->i2c, 0x75, &reg_value_75);
    if (reg_value_75 != expected_value_75) {
        // 处理错误
        printk("Error: Register 0x75 value mismatch, expected 0x%x, got 0x%x\n", expected_value_75, reg_value_75);
    }

    // 步骤12: 若使用XY量化，检查寄存器0x73和0x74
    // 假设使用XY量化
    uint8_t expected_values_73_74[] = {0x99, 0x02};
    for (int i = 0; i < 2; i++) {
        uint8_t reg_value;
        i2c_reg_read_byte_dt(&cfg->i2c, 0x73 + i, &reg_value);
        if (reg_value != expected_values_73_74[i]) {
            // 处理错误
            printk("Error: Register 0x%x value mismatch, expected 0x%x, got 0x%x\n", 0x73 + i, expected_values_73_74[i], reg_value);
        }
    }

    // 步骤13: 若使用突发模式，向寄存器0x1C写入0x10
    // 假设使用突发模式
    value = 0x10;
    i2c_reg_write_byte_dt(&cfg->i2c, 0x1C, value);

    // 步骤14: 从寄存器0x02、0x03和0x04读取一次数据
    uint8_t motion, delta_x, delta_y;
    i2c_reg_read_byte_dt(&cfg->i2c, 0x02, &motion);
    i2c_reg_read_byte_dt(&cfg->i2c, 0x03, &delta_x);
    i2c_reg_read_byte_dt(&cfg->i2c, 0x04, &delta_y);

    // 步骤15: 检查0x1a的值是否为0x00
    uint8_t expected_value_1a = 0x00;
    uint8_t reg_value_1a;
    i2c_reg_read_byte_dt(&cfg->i2c, 0x1a, &reg_value_1a);
    if (reg_value_1a != expected_value_1a) {
        // 处理错误
        printk("Error: Register 0x1a value mismatch, expected 0x%x, got 0x%x\n", expected_value_1a, reg_value_1a);
    }
}

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

    // 执行上电序列
    touchpad_power_up_sequence(cfg);

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
