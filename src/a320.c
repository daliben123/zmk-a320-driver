#define DT_DRV_COMPAT avago_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

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

/* 工作队列处理函数 (读取数据并清零寄存器) */
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
    data->x_delta = (x < 128) ? x : x - 256;
    data->y_delta = (y < 128) ? y : y - 256;
    
    LOG_DBG("检测到位移: dx=%d, dy=%d", data->x_delta, data->y_delta);

    // 写入Motion寄存器清除位移数据（符合A320手册要求）
    // 写入任意值均可清除MOT标志和Delta_X/Delta_Y寄存器
    a320_write_reg(dev, Motion, 0x00);
}

/* 中断服务函数 (立即响应中断) */
static void a320_motion_isr(const struct device *dev,
                           struct gpio_callback *cb, uint32_t pins) {
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, motion_cb);
    k_work_submit(&data->work);
}

static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    return 0;
}

static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct a320_data *data = dev->data;
    // 临时保存当前值用于返回，随后清零缓存
    int x = data->x_delta;
    int y = data->y_delta;
    
    switch (chan) {
    case SENSOR_CHAN_POS_DX:  // X位移
        val->val1 = x;
        data->x_delta = 0;  // 读取后清零缓存
        return 0;
    case SENSOR_CHAN_POS_DY:  // Y位移
        val->val1 = y;
        data->y_delta = 0;  // 读取后清零缓存
        return 0;
    case SENSOR_CHAN_AMBIENT_TEMP:  // MAIN.C适配点
        val->val1 = y;
        val->val2 = x;
        data->x_delta = 0;  // 读取后清零缓存
        data->y_delta = 0;
        return 0;
    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 完整的初始化序列 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    
    data->dev = dev;
    
    // 初始化I2C总线
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C总线 %s 未就绪!", cfg->bus.bus->name);
        return -ENODEV;
    }
    
    //==== 触控板硬复位序列（基于电路图GPIO）====
    // 配置关断引脚（GP24）
    if (cfg->shutdown_gpio.port != NULL) {
        if (!device_is_ready(cfg->shutdown_gpio.port)) {
            LOG_ERR("关断GPIO设备未就绪");
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->shutdown_gpio, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&cfg->shutdown_gpio, 0); // 拉低关断触控板
        k_msleep(10); // 增加延迟与Pico代码一致
    }

    // 配置复位引脚（GP16）
    if (cfg->reset_gpio.port != NULL) {
        if (!device_is_ready(cfg->reset_gpio.port)) {
            LOG_ERR("复位GPIO设备未就绪");
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&cfg->reset_gpio, 0); // 拉低复位
        k_msleep(100); // 增加延迟与Pico代码一致
        gpio_pin_set_dt(&cfg->reset_gpio, 1); // 释放复位
        k_msleep(10); // 增加延迟与Pico代码一致
    } else {
        // 如果没有复位引脚，添加一个默认延迟
        k_msleep(100);
    }
    
    //==== 设备通信验证 ====
    int pid = a320_read_reg(dev, Product_ID);
    int rid = a320_read_reg(dev, Revision_ID);
    
    if (pid < 0 || rid < 0) {
        LOG_ERR("设备ID读取失败");
        return -EIO;
    }
    
    // 根据A320数据手册调整验证值
    if (pid != 0x83 || rid != 0x01) {
        LOG_WRN("非标准A320设备: PID=0x%02X, RID=0x%02X", pid, rid);
        // 允许非标准设备继续初始化，提高兼容性
    } else {
        LOG_INF("A320初始化成功: PID=0x%02X, RID=0x%02X", pid, rid);
    }
    
    //==== 中断配置（使用GP22）====
    if (cfg->motion_gpio.port != NULL) {
        if (!device_is_ready(cfg->motion_gpio.port)) {
            LOG_ERR("动作检测GPIO设备未就绪");
            return -ENODEV;
        }
        
        // 配置为输入模式（电路图无上拉电阻，启用内部上拉）
        gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT | GPIO_PULL_UP);
        
        // 初始化回调
        gpio_init_callback(&data->motion_cb, a320_motion_isr, 
                          BIT(cfg->motion_gpio.pin));
        
        if (gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb) < 0) {
            LOG_ERR("回调添加失败");
            return -EIO;
        }
        
        // 修改为下降沿触发（与Pico代码一致）
        gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_INACTIVE);
    }
    
    // 初始化工作队列
    k_work_init(&data->work, a320_work_handler);
    
    // 清空初始数据
    data->x_delta = 0;
    data->y_delta = 0;
    
    LOG_INF("触控板驱动初始化完成");
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
    struct a320_data a3200_data_;                                                            \
    static const struct a320_config a3200_cfg_ = {                                           \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                             \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}),                           \
        .shutdown_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, shutdown_gpios, {0})                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a3200_data_, &a3200_cfg_,            \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
