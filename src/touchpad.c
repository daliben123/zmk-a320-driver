#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/gestures.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/mouse_movement.h>
#include <zmk/events/mouse_scroll.h>

LOG_MODULE_REGISTER(trackpad, CONFIG_SENSOR_LOG_LEVEL);

#define TRACKPAD_I2C_ADDR 0x3B

#define REG_PID        0x00
#define REG_REV        0x01
#define REG_MOTION     0x02
#define REG_DELTA_X    0x03
#define REG_DELTA_Y    0x04
#define REG_DELTA_XY_H 0x05
#define REG_CONFIG     0x11
#define REG_OBSERV     0x2E
#define REG_MBURST     0x42

#define BIT_MOTION_MOT (1 << 7)
#define BIT_MOTION_OVF (1 << 4)

#define SCROLL_SPEED_DIVIDER   6
#define BOOST_SPEED_MULTIPLIER 2
#define SLOW_SPEED_DIVIDER     2

struct trackpad_config {
    const struct device *i2c;
    uint8_t i2c_addr;
    gpio_pin_t shutdown_pin;
    gpio_pin_t motion_pin;
    gpio_pin_t reset_pin;
    gpio_dt_spec blk_kbd;
    gpio_dt_spec blk_tp;
};

struct trackpad_data {
    struct k_work work;
    bool is_scroll_mode;
    bool is_boost_mode;
    bool is_slow_mode;
    int16_t x_accum;
    int16_t y_accum;
};

static int trackpad_i2c_read(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct trackpad_config *cfg = dev->config;
    uint8_t buf[1] = {reg};
    int err;

    err = i2c_write(cfg->i2c, buf, 1, cfg->i2c_addr);
    if (err) {
        LOG_ERR("I2C write error: %d", err);
        return err;
    }

    err = i2c_read(cfg->i2c, val, 1, cfg->i2c_addr);
    if (err) {
        LOG_ERR("I2C read error: %d", err);
        return err;
    }

    return 0;
}

static void trackpad_work_handler(struct k_work *work) {
    struct trackpad_data *data = CONTAINER_OF(work, struct trackpad_data, work);
    const struct device *dev = work->parent;
    uint8_t motion;
    int8_t x, y;
    int err;

    err = trackpad_i2c_read(dev, REG_MOTION, &motion);
    if (err) {
        LOG_WRN("Failed to read motion register");
        return;
    }

    if (motion & BIT_MOTION_MOT) {
        err = trackpad_i2c_read(dev, REG_DELTA_X, (uint8_t *)&x);
        if (err) return;

        err = trackpad_i2c_read(dev, REG_DELTA_Y, (uint8_t *)&y);
        if (err) return;

        if (data->is_scroll_mode) {
            struct zmk_event_header *eh;
            struct mouse_scroll *scroll;

            eh = new_mouse_scroll();
            scroll = get_mouse_scroll(eh);
            scroll->h = x / SCROLL_SPEED_DIVIDER;
            scroll->v = -y / SCROLL_SPEED_DIVIDER;
            ZMK_EVENT_RAISE(eh);
        } else {
            float speed_factor = 1.5f;
            if (data->is_boost_mode) {
                speed_factor *= BOOST_SPEED_MULTIPLIER;
            } else if (data->is_slow_mode) {
                speed_factor /= SLOW_SPEED_DIVIDER;
            }

            data->x_accum += (int16_t)(x * -speed_factor);
            data->y_accum += (int16_t)(y * speed_factor);

            // Threshold for sending movement events
            if (ABS(data->x_accum) > 1 || ABS(data->y_accum) > 1) {
                struct zmk_event_header *eh;
                struct mouse_movement *move;

                eh = new_mouse_movement();
                move = get_mouse_movement(eh);
                move->x = data->x_accum;
                move->y = data->y_accum;
                ZMK_EVENT_RAISE(eh);

                data->x_accum = 0;
                data->y_accum = 0;
            }
        }
    }

    // Schedule next poll
    k_work_submit_in(&data->work, K_MSEC(10));
}

static int trackpad_init(const struct device *dev) {
    struct trackpad_data *data = dev->data;
    const struct trackpad_config *cfg = dev->config;
    int err;

    if (!device_is_ready(cfg->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    // Initialize GPIO pins
    err = gpio_pin_configure_dt(&cfg->blk_kbd, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure BLK_KBD pin: %d", err);
        return err;
    }

    err = gpio_pin_configure_dt(&cfg->blk_tp, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure BLK_TP pin: %d", err);
        return err;
    }

    // Trackpad reset sequence
    err = gpio_pin_configure_dt(cfg->shutdown_pin, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure shutdown pin: %d", err);
        return err;
    }

    err = gpio_pin_configure_dt(cfg->motion_pin, GPIO_INPUT);
    if (err) {
        LOG_ERR("Failed to configure motion pin: %d", err);
        return err;
    }

    err = gpio_pin_configure_dt(cfg->reset_pin, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure reset pin: %d", err);
        return err;
    }

    k_msleep(10);
    gpio_pin_set_dt(cfg->shutdown_pin, 0);
    gpio_pin_set_dt(cfg->reset_pin, 0);
    k_msleep(100);
    gpio_pin_set_dt(cfg->reset_pin, 1);

    // Initialize work queue
    k_work_init(&data->work, trackpad_work_handler);
    k_work_submit(&data->work);

    LOG_INF("Trackpad driver initialized");
    return 0;
}

static int trackpad_set_mode(const struct device *dev, uint8_t mode, bool enable) {
    struct trackpad_data *data = dev->data;

    switch (mode) {
        case 0: // Scroll mode
            data->is_scroll_mode = enable;
            break;
        case 1: // Boost mode
            data->is_boost_mode = enable;
            break;
        case 2: // Slow mode
            data->is_slow_mode = enable;
            break;
        default:
            return -EINVAL;
    }

    return 0;
}

static const struct trackpad_driver_api trackpad_api = {
    .set_mode = trackpad_set_mode,
};

static struct trackpad_data trackpad_data = {
    .is_scroll_mode = false,
    .is_boost_mode = false,
    .is_slow_mode = false,
    .x_accum = 0,
    .y_accum = 0,
};

static const struct trackpad_config trackpad_config = {
    .i2c = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(trackpad))),
    .i2c_addr = TRACKPAD_I2C_ADDR,
    .shutdown_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(trackpad), shutdown_gpios),
    .motion_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(trackpad), motion_gpios),
    .reset_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(trackpad), reset_gpios),
    .blk_kbd = GPIO_DT_SPEC_GET(DT_NODELABEL(trackpad), blk_kbd_gpios),
    .blk_tp = GPIO_DT_SPEC_GET(DT_NODELABEL(trackpad), blk_tp_gpios),
};

DEVICE_DT_INST_DEFINE(0, trackpad_init, NULL, &trackpad_data, &trackpad_config,
                      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &trackpad_api);    
