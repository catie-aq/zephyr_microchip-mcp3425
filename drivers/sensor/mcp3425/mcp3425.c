/*
 * Copyright (c) CATIE 2023
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp3425

// #include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MCP3425, CONFIG_SENSOR_LOG_LEVEL);

/* ================================= MCP3425 DATA & CONFIG ================================= */

#define MCP3425_DEFAULT_CONFIG 0b00010100 // 14bits mode, continuous conversion mode

// required by sensor API: device's private data struct
struct mcp3425_data {
    int32_t voltage;
};

// required by sensor API: device's instance configuration struct
struct mcp3425_config {
    const struct i2c_dt_spec bus;
};

/* =================================== PRIVATE FUNCTIONS =================================== */

// static int mcp3425_reg_read(const struct device *dev, uint8_t start, uint8_t *buf, uint32_t size)
// {
//     const struct mcp3425_config *cfg = dev->config;
//
//     //!\ Careful with i2c burst functions, see warning in i2c API header. Could not work.
//     return i2c_burst_read_dt(&cfg->bus, start, buf, size);
// }

/* ================================= MCP3425 API FUNCTIONS ================================= */

// sample fetch: get latest value through i2c
static int mcp3425_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct mcp3425_data *data = dev->data;
    uint8_t buf[2];
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_VOLTAGE) {
        LOG_ERR("Unsupported sensor channel (0x%02X). MCP3425 is an ADC, so use "
                "\"SENSOR_CHAN_VOLTAGE\" only.",
                chan);
        return -ENOTSUP;
    }

    /* read signed 16 bit double-buffered register value */
    const struct mcp3425_config *cfg = dev->config;
    ret = i2c_read_dt(&cfg->bus, buf, 2);
    if (ret < 0) {
        return ret;
    }

    /* device's hot junction register is a signed int */
    data->voltage = (int32_t)(int16_t)(buf[0] << 8) | buf[1];

    /* 0.0625C resolution per LSB */
    data->voltage *= 62500;

    return 0;
}

// channel get: return the last stored value of the sample fetch
static int mcp3425_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct mcp3425_data *data = dev->data;

    if (chan != SENSOR_CHAN_VOLTAGE) {
        return -ENOTSUP;
    }

    val->val1 = data->voltage / 1000000;
    val->val2 = data->voltage % 1000000;

    return 0;
}

// required by sensor API: driver specific API functions (see "sensor_driver_api" struct in
// sensor.h)
static const struct sensor_driver_api mcp3425_api = {
    .sample_fetch = mcp3425_sample_fetch,
    .channel_get = mcp3425_channel_get,
};

/* =============================== MCP3425 INIT & DT DRIVER DEF ================================ */

// required by sensor API: driver init function
static int mcp3425_init(const struct device *dev) {
    const struct mcp3425_config *cfg = dev->config;
    uint8_t buf[1];
    int ret;

    // check if bus is ready
    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("MCP3425 i2c bus %s not ready", cfg->bus.bus->name);
        return -ENODEV;
    }

    // Send MCP3425 configuration
    buf[0] = MCP3425_DEFAULT_CONFIG;
    ret = i2c_write_dt(&cfg->bus, buf, 1);
    LOG_INF("MCP3425 config: 0x%02X at addr 0x%02X", buf[0], (&cfg->bus)->addr);

    return ret;
}

// required by driver API: specific DT define process for MCP3425 sensor
#define MCP3425_DEFINE(id)                                                                                             \
    static struct mcp3425_data mcp3425_data_##id;                                                                      \
                                                                                                                       \
    static const struct mcp3425_config mcp3425_config_##id = {                                                         \
        .bus = I2C_DT_SPEC_INST_GET(id),                                                                               \
    };                                                                                                                 \
                                                                                                                       \
    SENSOR_DEVICE_DT_INST_DEFINE(id,                                                                                   \
            mcp3425_init,                                                                                              \
            NULL,                                                                                                      \
            &mcp3425_data_##id,                                                                                        \
            &mcp3425_config_##id,                                                                                      \
            POST_KERNEL,                                                                                               \
            CONFIG_SENSOR_INIT_PRIORITY,                                                                               \
            &mcp3425_api);

DT_INST_FOREACH_STATUS_OKAY(MCP3425_DEFINE)
