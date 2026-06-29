/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT alps_ec11

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include "ec11.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(EC11, CONFIG_SENSOR_LOG_LEVEL);

static inline int setup_int(const struct device *dev, bool enable) {
    const struct ec11_config *cfg = dev->config;
    int ret;

    ret = gpio_pin_interrupt_configure_dt(&cfg->a, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE);
    if (ret) {
        LOG_WRN("Unable to set A pin GPIO interrupt");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&cfg->b, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE);
    if (ret) {
        LOG_WRN("Unable to set B pin GPIO interrupt");
    }

    return ret;
}

/* Both edges are decoded inline in the ISR. We deliberately do NOT disable the
 * interrupts here: this encoder bounces with edges only microseconds apart, and
 * disabling around a deferred work item would drop those edges and break the
 * +1/-1 bounce cancellation in ec11_handle_edge(). The decode is cheap (two pin
 * reads + a table lookup), so running it in the ISR is fine. */
static void ec11_a_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                 uint32_t pins) {
    struct ec11_data *drv_data = CONTAINER_OF(cb, struct ec11_data, a_gpio_cb);

    ec11_handle_edge(drv_data->dev);
}

static void ec11_b_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                 uint32_t pins) {
    struct ec11_data *drv_data = CONTAINER_OF(cb, struct ec11_data, b_gpio_cb);

    ec11_handle_edge(drv_data->dev);
}

int ec11_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                     sensor_trigger_handler_t handler) {
    struct ec11_data *drv_data = dev->data;

    drv_data->trigger = trig;
    drv_data->handler = handler;

    return 0;
}

int ec11_init_interrupt(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    drv_data->dev = dev;

    /* setup gpio interrupt */
    gpio_init_callback(&drv_data->a_gpio_cb, ec11_a_gpio_callback, BIT(drv_cfg->a.pin));

    if (gpio_add_callback(drv_cfg->a.port, &drv_data->a_gpio_cb) < 0) {
        LOG_DBG("Failed to set A callback!");
        return -EIO;
    }

    gpio_init_callback(&drv_data->b_gpio_cb, ec11_b_gpio_callback, BIT(drv_cfg->b.pin));

    if (gpio_add_callback(drv_cfg->b.port, &drv_data->b_gpio_cb) < 0) {
        LOG_DBG("Failed to set B callback!");
        return -EIO;
    }

    /* Interrupts stay enabled for the lifetime of the device. Edges that arrive
     * before a handler is registered just accumulate into drv_data->pulses. */
    return setup_int(dev, true);
}
