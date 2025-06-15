#define DT_DRV_COMPAT avago_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief 从A320传感器读取指定寄存器的值
 * 
 * @param dev 设备指针
 * @param reg_addr 寄存器地址
 * @return int 成功时返回寄存器值，失败时返回错误码
 */
static int a320_read_reg(const struct device *dev, uint8_t reg_addr) {
    const struct a320_config *cfg = dev->config;
    uint8_t value = 0;

    if (i2c_reg_read_byte_dt(&cfg->bus, reg_addr, &value) == 0) {
        return value;
    }

    LOG_ERR("Failed to read register 0x%x", reg_addr);
    return -EIO;
}

/**
 * @brief 从传感器获取采样数据
 * 
 * @param dev 设备指针
 * @param chan 传感器通道
 * @return int 成功返回0，失败返回错误码
 */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_DX_DY) {
        return -ENOTSUP;
    }

    // 这里可以添加实际采样逻辑
    return 0;
}

/**
 * @brief 获取指定通道的传感器数据
 * 
 * @param dev 设备指针
 * @param chan 传感器通道
 * @param val 存储传感器数据的结构
 * @return int 成功返回0，失败返回错误码
 */
static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    int ifmotion, ovflow, dx, dy;

    if (chan != SENSOR_CHAN_POS_DX_DY) {
        return -ENOTSUP;
    }

    // 读取运动状态寄存器
    ifmotion = a320_read_reg(dev, MOTION);
    if (ifmotion < 0) {
        return ifmotion;
    }

    // 读取状态寄存器（原代码中重复读取Motion，可能是错误）
    ovflow = a320_read_reg(dev, OBSERVATION);
    if (ovflow < 0) {
        return ovflow;
    }

    // 检查是否有有效运动数据且未溢出
    if ((ifmotion & BIT_MOTION_MOT) && !(ovflow & BIT_MOTION_OVF)) {
        dx = a320_read_reg(dev, DELTA_X);
        dy = a320_read_reg(dev, DELTA_Y);

        if (dx < 0 || dy < 0) {
            return -EIO;
        }

        // 存储X和Y方向的移动量
        val->val1 = dx;
        val->val2 = dy;
        LOG_DBG("X movement: %d, Y movement: %d", val->val1, val->val2);
        return 0;
    } else {
        // 没有检测到运动或数据溢出，返回0
        val->val1 = 0;
        val->val2 = 0;
        return 0;
    }
}

/**
 * @brief A320传感器驱动API结构
 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/**
 * @brief 初始化A320传感器设备
 * 
 * @param dev 设备指针
 * @return int 成功返回0，失败返回错误码
 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    
    // 检查I2C总线是否就绪
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C bus %s is not ready!", cfg->bus.bus->name);
        return -EINVAL;
    }

    // 读取初始值以清除可能的旧数据
    a320_read_reg(dev, MOTION);
    a320_read_reg(dev, OBSERVATION);
    a320_read_reg(dev, DELTA_X);
    a320_read_reg(dev, DELTA_Y);

    LOG_DBG("A320 initialized successfully, ready to read data");
    return 0;
}

/**
 * @brief 为每个设备树实例定义A320设备
 */
#define A320_DEFINE(inst)                                                                          \
    struct a320_data a320_data_##inst;                                                            \
    static const struct a320_config a320_cfg_##inst = {.bus = I2C_DT_SPEC_INST_GET(inst)};        \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_cfg_##inst,            \
                          POST_KERNEL, 60, &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)    