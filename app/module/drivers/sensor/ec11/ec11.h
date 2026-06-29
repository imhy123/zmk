/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

struct ec11_config {
    const struct gpio_dt_spec a;
    const struct gpio_dt_spec b;

    const uint16_t steps;
    const uint8_t resolution;
    const uint8_t pulses_per_detent;
};

struct ec11_data {
    uint8_t ab_state;
    /* Completed detents pending a channel_get(); written from the ISR. */
    int8_t pulses;
    /* Valid quadrature transitions accumulated toward the next detent.
     * Contact bounce produces +1/-1 pairs that cancel here. */
    int8_t accum;

#ifdef CONFIG_EC11_TRIGGER
    struct gpio_callback a_gpio_cb;
    struct gpio_callback b_gpio_cb;
    const struct device *dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;
#endif /* CONFIG_EC11_TRIGGER */
};

/* Decode one GPIO edge and accumulate detents. Runs in ISR context (or from
 * sample_fetch when triggers are disabled). Never disables interrupts, so the
 * fast bounce edges are all seen and cancel out in accum. */
void ec11_handle_edge(const struct device *dev);

#ifdef CONFIG_EC11_TRIGGER

int ec11_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                     sensor_trigger_handler_t handler);

int ec11_init_interrupt(const struct device *dev);
#endif
