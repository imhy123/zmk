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
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "ec11.h"

#define FULL_ROTATION 360

LOG_MODULE_REGISTER(EC11, CONFIG_SENSOR_LOG_LEVEL);

static int ec11_get_ab_state(const struct device *dev) {
    const struct ec11_config *drv_cfg = dev->config;

    return (gpio_pin_get_dt(&drv_cfg->a) << 1) | gpio_pin_get_dt(&drv_cfg->b);
}

void ec11_handle_edge(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    uint8_t val = ec11_get_ab_state(dev);
    int8_t delta;

    switch (val | (drv_data->ab_state << 2)) {
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
        delta = -1;
        break;
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        delta = 1;
        break;
    default:
        /* No change, or a double jump we cannot resolve (should not happen now
         * that interrupts are never disabled). Resync state and bail. */
        drv_data->ab_state = val;
        return;
    }

    drv_data->ab_state = val;

    /* A detent is `pulses_per_detent` transitions in one direction. Bounce at
     * the switch point alternates +1/-1, so accum oscillates and never reaches
     * the threshold -- the bounce is filtered without any time-based debounce. */
    drv_data->accum += delta;

    bool detent = false;
    if (drv_data->accum >= (int8_t)drv_cfg->pulses_per_detent) {
        drv_data->pulses += 1;
        drv_data->accum = 0;
        detent = true;
    } else if (drv_data->accum <= -(int8_t)drv_cfg->pulses_per_detent) {
        drv_data->pulses -= 1;
        drv_data->accum = 0;
        detent = true;
    }

#ifdef CONFIG_EC11_TRIGGER
    if (detent && drv_data->handler) {
        /* In ISR context, zmk_sensors_trigger_handler just sets a pending bit
         * and submits its own work item, so bursts of detents coalesce. */
        drv_data->handler(dev, drv_data->trigger);
    }
#else
    ARG_UNUSED(detent);
#endif
}

static int ec11_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ROTATION);

#ifndef CONFIG_EC11_TRIGGER
    /* No interrupts: decode on demand. With triggers enabled the decoding is
     * already done in the ISR and this is a no-op. */
    ec11_handle_edge(dev);
#endif

    return 0;
}

static int ec11_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    /* pulses is also written from the ISR; read-and-clear atomically. */
    unsigned int key = irq_lock();
    int32_t pulses = drv_data->pulses;
    drv_data->pulses = 0;
    irq_unlock(key);

    if (drv_cfg->steps > 0) {
        /* pulses counts detents; steps is detents-per-rotation, so this yields
         * degrees consumed by behavior_sensor_rotate (360 / triggers_per_rotation). */
        val->val1 = (pulses * FULL_ROTATION) / drv_cfg->steps;
        val->val2 = (pulses * FULL_ROTATION) % drv_cfg->steps;
        if (val->val2 != 0) {
            val->val2 *= 1000000;
            val->val2 /= drv_cfg->steps;
        }
    } else {
        /* Legacy: report detents directly in val2 (val1 == 0 path). */
        val->val1 = 0;
        val->val2 = pulses;
    }

    return 0;
}

static const struct sensor_driver_api ec11_driver_api = {
#ifdef CONFIG_EC11_TRIGGER
    .trigger_set = ec11_trigger_set,
#endif
    .sample_fetch = ec11_sample_fetch,
    .channel_get = ec11_channel_get,
};

int ec11_init(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    LOG_DBG("A: %s %d B: %s %d resolution %d", drv_cfg->a.port->name, drv_cfg->a.pin,
            drv_cfg->b.port->name, drv_cfg->b.pin, drv_cfg->resolution);

    if (!device_is_ready(drv_cfg->a.port)) {
        LOG_ERR("A GPIO device is not ready");
        return -EINVAL;
    }

    if (!device_is_ready(drv_cfg->b.port)) {
        LOG_ERR("B GPIO device is not ready");
        return -EINVAL;
    }

    if (gpio_pin_configure_dt(&drv_cfg->a, GPIO_INPUT)) {
        LOG_DBG("Failed to configure A pin");
        return -EIO;
    }

    if (gpio_pin_configure_dt(&drv_cfg->b, GPIO_INPUT)) {
        LOG_DBG("Failed to configure B pin");
        return -EIO;
    }

#ifdef CONFIG_EC11_TRIGGER
    if (ec11_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }
#endif

    drv_data->ab_state = ec11_get_ab_state(dev);
    drv_data->accum = 0;

    return 0;
}

#define EC11_INST(n)                                                                               \
    static struct ec11_data ec11_data_##n;                                                         \
    static const struct ec11_config ec11_cfg_##n = {                                               \
        .a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                                    \
        .b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                                    \
        .resolution = DT_INST_PROP_OR(n, resolution, 1),                                           \
        .steps = DT_INST_PROP_OR(n, steps, 0),                                                     \
        .pulses_per_detent = DT_INST_PROP_OR(n, pulses_per_detent, 1),                             \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, ec11_init, NULL, &ec11_data_##n, &ec11_cfg_##n, POST_KERNEL,          \
                          CONFIG_SENSOR_INIT_PRIORITY, &ec11_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EC11_INST)
