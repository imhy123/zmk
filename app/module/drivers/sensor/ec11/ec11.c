/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT alps_ec11

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "ec11.h"

#define FULL_ROTATION 360

LOG_MODULE_REGISTER(EC11, CONFIG_SENSOR_LOG_LEVEL);

enum { EC11_EVT_STEP = 0, EC11_EVT_EMIT, EC11_EVT_CODIR, EC11_EVT_DROP };

#if IS_ENABLED(CONFIG_EC11_DEBUG_TRACE)
struct ec11_trace_rec {
    uint32_t t_us;
    uint8_t prev;
    uint8_t curr;
    uint8_t evt;
    int8_t step;
    int8_t accum;
    int8_t dir;
    int8_t pulses;
};

K_MSGQ_DEFINE(ec11_trace_q, sizeof(struct ec11_trace_rec), 256, 4);

static void ec11_trace_push(uint32_t t_us, uint8_t prev, uint8_t curr, int8_t step, int8_t accum,
                            int8_t dir, int8_t pulses, uint8_t evt) {
    struct ec11_trace_rec r = {.t_us = t_us,
                               .prev = prev,
                               .curr = curr,
                               .evt = evt,
                               .step = step,
                               .accum = accum,
                               .dir = dir,
                               .pulses = pulses};
    /* ISR-safe, never blocks: drop the record if the queue is full. */
    k_msgq_put(&ec11_trace_q, &r, K_NO_WAIT);
}

static void ec11_trace_thread(void *a, void *b, void *c) {
    static const char *const names[] = {"step", "EMIT", "CODIR", "drop"};
    struct ec11_trace_rec r;

    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1) {
        k_msgq_get(&ec11_trace_q, &r, K_FOREVER);
        LOG_INF("t=%u prev=%u curr=%u step=%d accum=%d dir=%d pulses=%d %s", r.t_us, r.prev, r.curr,
                r.step, r.accum, r.dir, r.pulses, names[r.evt]);
    }
}

K_THREAD_DEFINE(ec11_trace_tid, 1024, ec11_trace_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

#define EC11_TRACE(...) ec11_trace_push(__VA_ARGS__)
#else
#define EC11_TRACE(...)
#endif /* CONFIG_EC11_DEBUG_TRACE */

static int ec11_get_ab_state(const struct device *dev) {
    const struct ec11_config *drv_cfg = dev->config;

    return (gpio_pin_get_dt(&drv_cfg->a) << 1) | gpio_pin_get_dt(&drv_cfg->b);
}

void ec11_handle_edge(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    uint32_t now = k_cycle_get_32();
    uint8_t prev = drv_data->ab_state;
    uint8_t val = ec11_get_ab_state(dev);
    int8_t delta;

    switch (val | (prev << 2)) {
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
        if (val == prev) {
            return; /* redundant read, no change */
        }
        /* 2-step jump: both A and B differ, so two edges of one detent collapsed
         * into a single read (this encoder does it at every other detent). The
         * direction is ambiguous from the states alone, but it is two
         * transitions in whatever way we were already turning -- compensate the
         * otherwise-lost detent using the committed direction. */
        if (!drv_cfg->filter_jump_compensate || drv_data->dir == 0) {
            drv_data->ab_state = val;
            EC11_TRACE(k_cyc_to_us_floor32(now), prev, val, 0, drv_data->accum, drv_data->dir,
                       drv_data->pulses, EC11_EVT_DROP);
            return;
        }
        delta = 2 * drv_data->dir;
        break;
    }

    drv_data->ab_state = val;

    /* A detent is `pulses_per_detent` transitions in one direction. Bounce at
     * the switch point alternates +1/-1, so accum oscillates and never reaches
     * the threshold -- the bounce is filtered without dropping any edge. */
    drv_data->accum += delta;
    int8_t accum_shown __maybe_unused = drv_data->accum;

    /* Remember when we last moved in the committed direction. A reverse detent
     * that completes within filter_reverse_guard_us of this is a glitch (a real
     * reversal always has a velocity-through-zero time gap). */
    if (drv_data->dir == 0 || (delta < 0) == (drv_data->dir < 0)) {
        drv_data->t_codir = now;
    }

    const int8_t det = (int8_t)drv_cfg->pulses_per_detent;
    bool detent = false;
    uint8_t evt __maybe_unused = EC11_EVT_STEP;

    if (drv_data->accum >= det || drv_data->accum <= -det) {
        int8_t want = (drv_data->accum >= det) ? 1 : -1; /* direction this detent wants */
        bool glitch = drv_data->dir != 0 && want != drv_data->dir && drv_cfg->filter_reverse_guard_us > 0 &&
                      k_cyc_to_us_floor32(now - drv_data->t_codir) < drv_cfg->filter_reverse_guard_us;
        drv_data->accum = 0;

        if (!glitch) {
            drv_data->pulses += want;
            drv_data->dir = want;
            detent = true;
            evt = EC11_EVT_EMIT;
        } else if (drv_cfg->filter_reverse_as_codir &&
                   k_cyc_to_us_floor32(now - drv_data->t_last_emit) >= drv_cfg->filter_codir_guard_us) {
            /* A glitch this long after the last detent is a genuinely missed
             * same-direction detent -- recover it in the committed direction so
             * the count does not lag. A glitch sooner than filter_codir_guard_us is just
             * the just-emitted detent's settling chatter; dropping it keeps one
             * physical detent from emitting twice. */
            drv_data->pulses += drv_data->dir;
            detent = true;
            evt = EC11_EVT_CODIR;
        } else {
            evt = EC11_EVT_DROP; /* glitch dropped (chatter echo, or codir disabled) */
        }

        if (detent) {
            drv_data->t_last_emit = now;
        }
    }

    EC11_TRACE(k_cyc_to_us_floor32(now), prev, val, delta, accum_shown, drv_data->dir,
               drv_data->pulses, evt);

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
    drv_data->dir = 0;
    drv_data->t_codir = 0;
    drv_data->t_last_emit = 0;

    return 0;
}

#define EC11_INST(n)                                                                               \
    static struct ec11_data ec11_data_##n;                                                         \
    static const struct ec11_config ec11_cfg_##n = {                                               \
        .a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                                    \
        .b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                                    \
        .resolution = DT_INST_PROP_OR(n, resolution, 1),                                           \
        .steps = DT_INST_PROP_OR(n, steps, 0),                                                     \
        .pulses_per_detent = DT_INST_PROP_OR(n, pulses_per_detent, 2),                             \
        .filter_reverse_guard_us = DT_INST_PROP_OR(n, filter_reverse_guard_us, 800),                             \
        .filter_reverse_as_codir = DT_INST_PROP_OR(n, filter_reverse_as_codir, 0),                 \
        .filter_codir_guard_us = DT_INST_PROP_OR(n, filter_codir_guard_us, 3000),                                \
        .filter_jump_compensate = DT_INST_PROP_OR(n, filter_jump_compensate, 1),                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, ec11_init, NULL, &ec11_data_##n, &ec11_cfg_##n, POST_KERNEL,          \
                          CONFIG_SENSOR_INIT_PRIORITY, &ec11_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EC11_INST)
