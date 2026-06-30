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
    /* If >0, a reverse detent that completes within this many microseconds of
     * the last same-as-committed-direction transition is treated as a glitch.
     * 0 disables the filter. Defaults to 800. */
    const uint32_t filter_reverse_guard_us;
    /* When a glitch is caught by filter_reverse_guard_us: if nonzero, emit a
     * detent in the committed direction (the glitch was misdecoded
     * same-direction motion); if 0, drop it entirely. Defaults to 0. */
    const uint8_t filter_reverse_as_codir;
    /* Minimum time since the last emitted detent before an *inferred* detent may
     * fire -- both the same-direction (codir) recovery and the 2-step-jump
     * compensation below. An inferred emit sooner than this is the just-emitted
     * detent's settling chatter (or a burst of collapsed reads within one
     * detent), so it is dropped to keep one physical detent from emitting more
     * than once. 0 disables this gate. Defaults to 3000. */
    const uint32_t filter_codir_guard_us;
    /* When two edges of a detent collapse into one read (a 2-step jump, both A
     * and B differ), the direction is ambiguous from the states alone. If
     * nonzero, compensate the otherwise-lost detent by resolving the jump as two
     * transitions in the committed direction. If 0, drop it. Defaults to 1. */
    const uint8_t filter_jump_compensate;
};

struct ec11_data {
    uint8_t ab_state;
    /* Completed detents pending a channel_get(); written from the ISR. */
    int8_t pulses;
    /* Valid quadrature transitions accumulated toward the next detent.
     * Contact bounce produces +1/-1 pairs that cancel here. */
    int8_t accum;
    /* Committed rotation direction of the last emitted detent (+1/-1/0). */
    int8_t dir;
    /* Cycle timestamp of the last transition that went in `dir`. */
    uint32_t t_codir;
    /* Cycle timestamp of the last emitted detent. */
    uint32_t t_last_emit;

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
