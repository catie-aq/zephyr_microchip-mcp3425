/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
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
    struct sensor_value mcp3425_value;

    // GPIO led
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    // MCP3425 ADC sensor
    if (mcp3425_sensor == NULL) {
        printf("Device MCP3425 not found.\n");
        return 0;
    }

    if (!device_is_ready(mcp3425_sensor)) {
        printf("Device %s is not ready.\n", mcp3425_sensor->name);
        return 0;
    }

    LOG_DBG("test log dbg"); // level 4 + zephyr kernel dbg
    LOG_INF("test log info"); // level 3
    LOG_WRN("test log warning"); // level 2
    LOG_ERR("test log error"); // level 1

    while (1) {

        // toggle GPIO
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;
        }

        // fetch and get MCP3425 ADC value
        ret = sensor_sample_fetch(mcp3425_sensor);
        if (ret != 0) {
            printf("sensor_sample_fetch error: %d\n", ret);
            break;
        }

        ret = sensor_channel_get(mcp3425_sensor, SENSOR_CHAN_VOLTAGE, &mcp3425_value);
        if (ret != 0) {
            printf("sensor_channel_get error: %d\n", ret);
            break;
        }

        //        LOG_INF("ADC value: %2.3fV", sensor_value_to_float(&mcp3425_value));
        LOG_INF("ADC value: %d.%dV", (&mcp3425_value)->val1, (&mcp3425_value)->val2);

        // k_sleep(K_SECONDS(2));
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
