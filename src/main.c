/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 2000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* The devicetree node identifier for the "mcp3425_zest_halfbridge" label */
#define MCP3425_NODE DT_NODELABEL(mcp3425_zest_halfbridge)

LOG_MODULE_REGISTER(Main);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *mcp3425_sensor = DEVICE_DT_GET(MCP3425_NODE);

int main(void) {
    int ret;

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    LOG_DBG("test log dbg"); // level 4 + zephyr kernel dbg
    LOG_INF("test log info"); // level 3
    LOG_WRN("test log warning"); // level 2
    LOG_ERR("test log error"); // level 1

    while (1) {
        ret = gpio_pin_toggle_dt(&led);

        if (ret < 0) {
            return 0;
        }

        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
