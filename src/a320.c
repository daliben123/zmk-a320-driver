#define DT_DRV_COMPAT avago_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>  // 新增：输入子系统头文件
#include <zephyr/logging/log.h>
#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

struct a320_data {
    const struct device *dev;
    struct gpio_callback motion_cb;
    struct k_work work;
    int16_t x_delta;
    int16_t y_delta;
};

struct a320_config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec shutdown_gpio;
};

static int a320_read_reg(const struct device *dev, uint8_t reg_addr) {
    const struct a320_config *cfg = dev->config;
    uint8_t value = 0;
    int ret;
    
    ret = i2c_reg_read_byte_dt(&cfg->bus, reg_addr, &value);
    if (ret < 0) {
        LOG_ERR("寄存器读取失败 0x%x (错误 %d)", reg_addr, ret);
        return ret;
    }
    
    return value;
}

static int a320_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value) {
    const struct a320_config *cfg = dev->config;
    int ret;
    
    ret = i2c_reg_write_byte_dt(&cfg->bus, reg_addr, value);
    if (ret < 0) {
        LOG_ERR("寄存器写入失败 0x%x (错误 %d)", reg_addr, ret);
    }
    return ret;
}

/* 工作队列处理函数 - 使用输入子系统上报事件 */
static void a320_work_handler(struct k_work *work) {
    struct a320_data *data = CONTAINER_OF(work, struct a320_data, work);
    const struct device *dev = data->dev;
    
    int motion = a320_read_reg(dev, Motion);
    if (motion < 0 || !(motion & BIT_MOTION_MOT)) {
        return;
    }
    
    int x = a320_read_reg(dev, Delta_X);
    int y = a320_read_reg(dev, Delta_Y);
    
    if (x < 0 || y < 0) {
        return;
    }
    
    // 转换为有符号位移
    int16_t x_delta = (x < 128) ? x : x - 256;
    int16_t y_delta = (y < 128) ? y : y - 256;
    
    LOG_DBG("检测到位移: dx=%d, dy=%d", x_delta, y_delta);

    // 使用输入子系统上报相对移动事件 [6](@ref)
    input_report_rel(dev, INPUT_REL_X, x_delta, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Y, y_delta, false, K_FOREVER);
    
    // 可选：上报同步事件，表示一个完整的输入事件帧结束
    input_report_key(dev, INPUT_BTN_TOUCH, 1, false, K_FOREVER);
    input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);

    // 写入Motion寄存器清除位移数据
    a320_write_reg(dev, Motion, 0x00);
}

/* 中断服务函数 */
static void a320_motion_isr(const struct device *dev,
                           struct gpio_callback *cb, uint32_t pins) {
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, motion_cb);
    k_work_submit(&data->work);
}

/* 输入子系统回调函数（可选，用于调试）*/
static void a320_input_cb(struct input_event *evt) {
    LOG_DBG("输入事件: type=%02x code=%03d value=%d", 
            evt->type, evt->code, evt->value);
}

// 注册输入事件回调 [6](@ref)
INPUT_LISTENER_CB_DEFINE(NULL, a320_input_cb);

/* 初始化函数 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    
    data->dev = dev;
    
    // 初始化I2C总线
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C总线 %s 未就绪!", cfg->bus.bus->name);
        return -ENODEV;
    }
    
    // 触控板硬复位序列
    if (cfg->shutdown_gpio.port != NULL) {
        if (!device_is_ready(cfg->shutdown_gpio.port)) {
            LOG_ERR("关断GPIO设备未就绪");
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->shutdown_gpio, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&cfg->shutdown_gpio, 0);
        k_msleep(10);
    }

    if (cfg->reset_gpio.port != NULL) {
        if (!device_is_ready(cfg->reset_gpio.port)) {
            LOG_ERR("复位GPIO设备未就绪");
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&cfg->reset_gpio, 0);
        k_msleep(100);
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_msleep(10);
    } else {
        k_msleep(100);
    }
    
    // 设备通信验证
    int pid = a320_read_reg(dev, Product_ID);
    int rid = a320_read_reg(dev, Revision_ID);
    
    if (pid < 0 || rid < 0) {
        LOG_ERR("设备ID读取失败");
        return -EIO;
    }
    
    if (pid != 0x83 || rid != 0x01) {
        LOG_WRN("非标准A320设备: PID=0x%02X, RID=0x%02X", pid, rid);
    } else {
        LOG_INF("A320初始化成功: PID=0x%02X, RID=0x%02X", pid, rid);
    }
    
    // 中断配置
    if (cfg->motion_gpio.port != NULL) {
        if (!device_is_ready(cfg->motion_gpio.port)) {
            LOG_ERR("动作检测GPIO设备未就绪");
            return -ENODEV;
        }
        
        gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT | GPIO_PULL_UP);
        
        gpio_init_callback(&data->motion_cb, a320_motion_isr, 
                          BIT(cfg->motion_gpio.pin));
        
        if (gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb) < 0) {
            LOG_ERR("回调添加失败");
            return -EIO;
        }
        
        gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_INACTIVE);
    }
    
    // 初始化工作队列
    k_work_init(&data->work, a320_work_handler);
    
    LOG_INF("A320触控板输入驱动初始化完成");
    return 0;
}

// 定义GPIO_DT_SPEC_INST_GET_OR宏
#ifndef GPIO_DT_SPEC_INST_GET_OR
#define GPIO_DT_SPEC_INST_GET_OR(inst, prop, default_value) \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop), \
               (GPIO_DT_SPEC_INST_GET(inst, prop)), \
               (default_value))
#endif

// 设备实例化宏
#define A320_DEFINE(inst)                                                                          \
    struct a320_data a320##inst##_data;                                                            \
    static const struct a320_config a320##inst##_cfg = {                                           \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                             \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}),                           \
        .shutdown_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, shutdown_gpios, {0})                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320##inst##_data, &a320##inst##_cfg,            \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);  // 注意：API设为NULL

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
