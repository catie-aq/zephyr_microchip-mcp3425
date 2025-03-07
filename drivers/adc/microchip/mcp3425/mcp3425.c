/*
 * Copyright (c) CATIE 2023
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp3425

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MCP3425, CONFIG_ADC_LOG_LEVEL);

/* ================================= MCP3425 DATA & CONFIG (could be in a .h) ============== */

#define MCP3425_DEFAULT_CONFIG     0b00010100 // 14bits mode, continuous conversion mode
#define MCP3425_VOLTAGE_DERIVATIVE 1.0029f    // Calibrate with a multimeter, approximately.

#define MCP3425_MAX_CHANNEL 1
#define MCP3425_INTERNAL_VOLTAGE_REFERENCE 2048 // in mV

#define MCP3425_CONF_12BITS 0b00 /* 240 SPS */
#define MCP3425_CONF_14BITS 0b01 /* 60 SPS */
#define MCP3425_CONF_16BITS 0b10 /* 15 SPS */

#define MCP3425_CONF_CONV_ONE_SHOT   0b0
#define MCP3425_CONF_CONV_CONTINUOUS 0b1

#define MCP3425_SHIFT_PGA     0    /* PGA Gain */
#define MCP3425_SHIFT_RESOL   2    /* Sample Rate / resolution */
#define MCP3425_SHIFT_CONV    4    /* Conversion mode */
#define MCP3425_SHIFT_RDY_BIT 7    /* ready bit */
#define MCP3425_MASK_RDY_BIT  0x80 /* ready bit mask */

#define MCP3425_DATA_VALID_MAX_RETRY 500

/* required by sensor API: device's private data struct */
struct mcp3425_data {
	uint8_t config_register;
	uint16_t *buffer;
};

/* required by sensor API: device's instance configuration struct */
struct mcp3425_config {
	const struct i2c_dt_spec bus;
	const int32_t pga_gain;   /* in xV/V. See binding for more infos */
	const int32_t resolution; /* in bits. See binding for more infos */
	const int one_shot_mode;
};

static int mcp3425_configuration(const struct device *dev)
{
	const struct mcp3425_config *cfg = dev->config;
	struct mcp3425_data *data = dev->data;
	int ret;

	/* setup resolution and lsb */
	uint8_t mcp3425_i2c_config_resolution_bits;
	switch (cfg->resolution) {
	case 12:
		LOG_DBG("12 bits mode.");
		mcp3425_i2c_config_resolution_bits = MCP3425_CONF_12BITS;
		break;
	case 14:
		LOG_DBG("14 bits mode (default).");
		mcp3425_i2c_config_resolution_bits = MCP3425_CONF_14BITS;
		break;
	case 16:
		LOG_DBG("16 bits mode.");
		mcp3425_i2c_config_resolution_bits = MCP3425_CONF_16BITS;
		break;
	default:
		LOG_ERR("Unrecognized resolution, this case can't happened. Check binding.");
		return -EINVAL;
	}

	LOG_DBG("PGA gain=%d, max input voltage is %dmV", cfg->pga_gain,
		MCP3425_INTERNAL_VOLTAGE_REFERENCE);

	/* todo: improve this part?
	 * setup conversion mode
	 */
	 uint8_t mcp3425_i2c_config_conversion_mode;
	if (cfg->one_shot_mode) {
		LOG_DBG("Conversion mode for %s is one-shot.", dev->name);
		LOG_DBG("Sample fetch may take more time (current max data retry is %d).",
			MCP3425_DATA_VALID_MAX_RETRY);
		mcp3425_i2c_config_conversion_mode = MCP3425_CONF_CONV_ONE_SHOT;
	} else {
		LOG_DBG("Conversion mode for %s is continuous", dev->name);
		mcp3425_i2c_config_conversion_mode = MCP3425_CONF_CONV_CONTINUOUS;
	}

	/* Send MCP3425 configuration */
	data->config_register = 0x00 | (mcp3425_i2c_config_conversion_mode << MCP3425_SHIFT_CONV) |
				(mcp3425_i2c_config_resolution_bits << MCP3425_SHIFT_RESOL) |
				((cfg->pga_gain >> 1) << MCP3425_SHIFT_PGA);
	ret = i2c_write_dt(&cfg->bus, &data->config_register, 1);
	LOG_DBG("Sent config 0x%02X to %s (ret=%d)", data->config_register, dev->name, ret);

	if (ret < 0) {
		LOG_ERR("Init fail for %s (i2c ret=%d) !", dev->name, ret);
		LOG_ERR("Is the device correctly connected, power up, and config with the correct "
			"address ?");
	}

	return 0;
}

static int mcp3425_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
	if (channel_cfg->channel_id > MCP3425_MAX_CHANNEL) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("unsupported channel reference '%d'", channel_cfg->reference);
		return -ENOTSUP;
	}

	switch(channel_cfg->gain) {
		case ADC_GAIN_1:
		case ADC_GAIN_2:
		case ADC_GAIN_4:
		case ADC_GAIN_8:
			break;
		default:
			LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
			return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'", channel_cfg->acquisition_time);
		return -ENOTSUP;
	}

	return mcp3425_configuration(dev);
}

static int mcp3425_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct mcp3425_config *cfg = dev->config;
	struct mcp3425_data *data = dev->data;
	static int16_t voltage_raw;
	static uint8_t data_is_not_ready;
	uint8_t buf[3];
	int ret;
	data->buffer = sequence->buffer;

	/* if in one shot mode, need to send config to trig a conversion */
	if (cfg->one_shot_mode) {
		static uint8_t one_shot_command;
		one_shot_command = ((data->config_register) |
				    MCP3425_MASK_RDY_BIT); /* writing RDY to wake up conversion */
		ret = i2c_write_dt(&cfg->bus, &one_shot_command, 1);
		if (ret < 0) {
			LOG_ERR("one shot fail for %s (i2c ret=%d) !", dev->name, ret);
			return ret;
		}
	}

	data_is_not_ready = 1;
	unsigned int max_retry = MCP3425_DATA_VALID_MAX_RETRY;

	while (data_is_not_ready && max_retry) {
		/* read signed 16 bit double-buffered register value */
		ret = i2c_read_dt(&cfg->bus, buf, 3);
		if (ret < 0) {
			LOG_ERR("Can't access driver %s anymore. No power ?", dev->name);
			return ret;
		}
		data_is_not_ready = ((buf[2] >> MCP3425_SHIFT_RDY_BIT) & 0x01);

		/* Todo: Maybe it could be better to wait instead of spamming the i2c.
		 * depending of the chosen ADC resolution, the time conversion is known.
		 */
		max_retry--;
		if (!max_retry) {
			LOG_WRN("After %d retry, data is still not ready for %s !",
				MCP3425_DATA_VALID_MAX_RETRY, dev->name);
		}
	}

	if (cfg->one_shot_mode) {
		LOG_DBG("One shot mode: get correct value after %d retry",
			MCP3425_DATA_VALID_MAX_RETRY - max_retry);
	}

	/* build raw voltage. In 12 and 14-bits modes, MSB is repeated by the ADC for direct int16
	 * support.
	 */
	voltage_raw = (int16_t)((buf[0] << 8) | buf[1]) >> 1;
	*data->buffer++ = voltage_raw;

	return 0;
}

static const struct adc_driver_api mcp3425_driver_api = {
	.channel_setup = mcp3425_channel_setup,
	.read = mcp3425_read,
	.ref_internal = MCP3425_INTERNAL_VOLTAGE_REFERENCE,
};

/* =============================== MCP3425 INIT & DT DRIVER DEF ================================ */

/* required by sensor API: driver init function */
static int mcp3425_init(const struct device *dev)
{
	const struct mcp3425_config *cfg = dev->config;

	/* check if bus is ready */
	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("i2c bus %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	return 0;
}

/* required by driver API: specific DT define process for each MCP3425 sensor instance */
#define MCP3425_DEFINE(inst)                                                                       \
	static struct mcp3425_data mcp3425_data_##inst;                                            \
	static const struct mcp3425_config mcp3425_config_##inst = {                               \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.pga_gain = DT_STRING_TOKEN(ADC_CHANNEL_DT_NODE(DT_DRV_INST(inst), 1), zephyr_gain), \
		.resolution = DT_PROP_OR(ADC_CHANNEL_DT_NODE(DT_DRV_INST(inst), 1), zephyr_resolution, 0), \
		.one_shot_mode = DT_INST_PROP(inst, one_shot_mode),                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mcp3425_init, NULL, &mcp3425_data_##inst,               \
				     &mcp3425_config_##inst, POST_KERNEL,                          \
				     CONFIG_ADC_INIT_PRIORITY, &mcp3425_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MCP3425_DEFINE)
