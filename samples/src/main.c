/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc/voltage_divider.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 2000

/* The devicetree node identifier for the "mcp3425" label */
#define MCP3425_NODE DT_NODELABEL(mcp3425)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), a0);
static const struct voltage_divider_dt_spec voltage_spec =
	VOLTAGE_DIVIDER_DT_SPEC_GET(DT_NODELABEL(voltage));

int main(void)
{
	int ret;
	uint16_t buf;
	int32_t val_mv = 0;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	if (!adc_is_ready_dt(&adc_channel)) {
		printf("Device %s is not ready.\n", adc_channel.dev->name);
		return 0;
	}

	ret = adc_channel_setup_dt(&adc_channel);
	if (ret != 0) {
		printf("sensor_sample_fetch error: %d\n", ret);
	}
	printk("ADC ref internal: %d\n", adc_ref_internal(adc_channel.dev));
	printk("ADC gain: %d\n", adc_channel.channel_cfg.gain);
	printk("ADC resolution: %d\n", adc_channel.resolution);
	int test = 1000; /* 1000mV */
	adc_gain_invert(adc_channel.channel_cfg.gain, &test);
	printk("ADC invertGain: %d\n", test);

	while (1) {
		(void)adc_sequence_init_dt(&adc_channel, &sequence);
		ret = adc_read_dt(&adc_channel, &sequence);

		if (ret != 0) {
			printf("sensor_channel_get error: %d\n", ret);
			break;
		}
		val_mv = (int32_t)buf;
		printk("RAW value: %d\n", val_mv);

		ret = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
		if (ret < 0) {
			printf("Raw to millivolts conversion error: %d\n", ret);
		} else {
			printk("ADC value: %d mV\n", val_mv);
			voltage_divider_scale_dt(&voltage_spec, &val_mv);
			printk("FULL value: %d mV\n", val_mv);
		}

		k_sleep(K_SECONDS(5));
	}
	return 0;
}
