#ifndef ZMK_INCLUDE_DRIVERS_SENSOR_TRACKPAD_H_
#define ZMK_INCLUDE_DRIVERS_SENSOR_TRACKPAD_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct trackpad_driver_api {
    int (*set_mode)(const struct device *dev, uint8_t mode, bool enable);
};

/**
 * @brief Set trackpad mode
 * 
 * @param dev Trackpad device
 * @param mode Mode to set (0=scroll, 1=boost, 2=slow)
 * @param enable Enable or disable the mode
 * @return int 0 on success, negative error code on failure
 */
static inline int trackpad_set_mode(const struct device *dev, uint8_t mode, bool enable) {
    const struct trackpad_driver_api *api = (const struct trackpad_driver_api *)dev->api;
    return api->set_mode(dev, mode, enable);
}

#ifdef __cplusplus
}
#endif

#endif /* ZMK_INCLUDE_DRIVERS_SENSOR_TRACKPAD_H_ */    
