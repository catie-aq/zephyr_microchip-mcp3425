/*
 * Copyright (c) 2025, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ADC_MCP3425_MCP3425_H_
#define ZEPHYR_DRIVERS_ADC_MCP3425_MCP3425_H_

#define MCP3425_DEFAULT_CONFIG     0b00010100 // 14bits mode, continuous conversion mode
#define MCP3425_VOLTAGE_DERIVATIVE 1.0029f    // Calibrate with a multimeter, approximately.

#define MCP3425_MAX_CHANNEL                1
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

#endif /* ZEPHYR_DRIVERS_ADC_MCP3425_MCP3425_H_ */
