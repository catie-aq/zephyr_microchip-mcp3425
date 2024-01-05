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

/* ================================= MCP3425 DATA & CONFIG (could be in a .h) ============== */

#define MCP3425_DEFAULT_CONFIG 0b00010100 // 14bits mode, continuous conversion mode
#define MCP3425_VOLTAGE_DERIVATIVE 1.0029f // Calibrate with a multimeter, approximately.

#define MCP3425_VOLTAGE_REFERENCE 2048 // in mV

#define MCP3425_LSB_12BITS 10000
#define MCP3425_LSB_14BITS 2500
#define MCP3425_LSB_16BITS 625

#define MCP3425_CONF_12BITS 0b00 // 240 SPS
#define MCP3425_CONF_14BITS 0b01 // 60 SPS
#define MCP3425_CONF_16BITS 0b10 // 15 SPS

#define MCP3425_CONF_PGA_1 0b00 // 1 V/V Gain
#define MCP3425_CONF_PGA_2 0b01 // 2 V/V Gain
#define MCP3425_CONF_PGA_4 0b10 // 4 V/V Gain
#define MCP3425_CONF_PGA_8 0b11 // 8 V/V Gain

#define MCP3425_CONF_CONV_ONE_SHOT 0b0
#define MCP3425_CONF_CONV_CONTINUOUS 0b1

#define MCP3425_SHIFT_PGA 0 // PGA Gain
#define MCP3425_SHIFT_RESOL 2 // Sample Rate / resolution
#define MCP3425_SHIFT_CONV 4 // Conversion mode

#define MCP3425_VOLTAGE_DECIMAL 10000000 // 10^(7)

// required by sensor API: device's private data struct
struct mcp3425_data {
    int32_t adc_value_lsb;
    int32_t voltage_uv; // in 10^(-7) volts or [uV]x0.1 !
};

// required by sensor API: device's instance configuration struct
struct mcp3425_config {
    const struct i2c_dt_spec bus;
    const int32_t adc_resolution; // in bits. See binding for more infos.
    const int32_t adc_pga_gain; // in xV/V. See binding for more infos.
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
    static int16_t voltage_raw;
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
        LOG_ERR("Can't access driver %s anymore. No power ?", dev->name);
        return ret;
    }

    /* build raw voltage */
    voltage_raw = (int16_t)((buf[0] << 8) | buf[1]);

    /* compute true voltage */
    data->voltage_uv = voltage_raw * data->adc_value_lsb / cfg->adc_pga_gain;

    return 0;
}

// channel get: return the last stored value of the sample fetch
static int mcp3425_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct mcp3425_data *data = dev->data;

    if (chan != SENSOR_CHAN_VOLTAGE) {
        LOG_ERR("This is a simple ADC. Only SENSOR_CHAN_VOLTAGE is supported.");
        return -ENOTSUP;
    }

    val->val1 = data->voltage_uv / MCP3425_VOLTAGE_DECIMAL; // integer part
    val->val2 = (data->voltage_uv % MCP3425_VOLTAGE_DECIMAL) / 10; // fractional part, must be 10^(-6) and not 10^(-7) !

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
    struct mcp3425_data *data = dev->data;
    uint8_t mcp3425_config_register[1];
    int ret;

    // check if bus is ready
    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("i2c bus %s not ready", cfg->bus.bus->name);
        return -ENODEV;
    }

    // setup resolution and lsb
    uint8_t mcp3425_i2c_config_resolution_bits;
    switch (cfg->adc_resolution) {
        case 16:
            LOG_DBG("16 bits mode.");
            data->adc_value_lsb = MCP3425_LSB_16BITS;
            mcp3425_i2c_config_resolution_bits = MCP3425_CONF_16BITS;
            break;
        case 12:
            LOG_DBG("12 bits mode.");
            data->adc_value_lsb = MCP3425_LSB_12BITS;
            mcp3425_i2c_config_resolution_bits = MCP3425_CONF_12BITS;
            break;
        default: // should never happen with current yaml binding configuration, thanks to the enum.
            LOG_WRN("resolution not recognized (%d bits), going to default mode (14bits).", cfg->adc_resolution);
        case 14:
            LOG_DBG("14 bits mode (default).");
            data->adc_value_lsb = MCP3425_LSB_14BITS;
            mcp3425_i2c_config_resolution_bits = MCP3425_CONF_14BITS;
            break;
    }

    LOG_INF("PGA gain=%d, max input voltage is %dmV.",
            cfg->adc_pga_gain,
            MCP3425_VOLTAGE_REFERENCE / cfg->adc_pga_gain);

    // Send MCP3425 configuration
    mcp3425_config_register[0] = 0x00 | (MCP3425_CONF_CONV_CONTINUOUS << MCP3425_SHIFT_CONV)
            | (mcp3425_i2c_config_resolution_bits << MCP3425_SHIFT_RESOL)
            | ((cfg->adc_pga_gain >> 1) << MCP3425_SHIFT_PGA);
    ret = i2c_write_dt(&cfg->bus, mcp3425_config_register, 1);
    LOG_INF("Sent config 0x%02X to %s (ret=%d)", mcp3425_config_register[0], dev->name, ret);

    if (ret < 0) {
        LOG_ERR("Init fail (i2c ret=%d) !", ret);
        LOG_ERR("Is the device correctly connected, power up, and config with the correct address ?");
    }

    return ret;
}

// required by driver API: specific DT define process for each MCP3425 sensor instance
#define MCP3425_DEFINE(id)                                                                                             \
    static struct mcp3425_data mcp3425_data_##id;                                                                      \
                                                                                                                       \
    static const struct mcp3425_config mcp3425_config_##id = {                                                         \
        .bus = I2C_DT_SPEC_INST_GET(id),                                                                               \
        .adc_resolution = DT_PROP(DT_DRV_INST(id), resolution),                                                        \
        .adc_pga_gain = DT_PROP(DT_DRV_INST(id), pga_gain),                                                            \
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
