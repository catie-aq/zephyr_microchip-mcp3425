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

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* The devicetree node identifier for the "mcp3425_zest_halfbridge" label */
#define MCP3425_NODE DT_NODELABEL(mcp34250)

LOG_MODULE_REGISTER(Main);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
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

	// GPIO led
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!adc_is_ready_dt(&adc_channel)) {
		printf("Device %s is not ready.\n", adc_channel.dev->name);
		return 0;
	}

	ret = adc_channel_setup_dt(&adc_channel);
	if (ret != 0) {
		printf("sensor_sample_fetch error: %d\n", ret);
	}
	LOG_INF("ADC ref internal: %d", adc_ref_internal(adc_channel.dev));
	LOG_INF("ADC gain: %d", adc_channel.channel_cfg.gain);
	LOG_INF("ADC resolution: %d", adc_channel.resolution);
	int test = 1000;
	adc_gain_invert(adc_channel.channel_cfg.gain, &test);
	LOG_INF("ADC invertGain: %d", test);

	while (1) {

		// toggle GPIO
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		(void)adc_sequence_init_dt(&adc_channel, &sequence);
		ret = adc_read_dt(&adc_channel, &sequence);

		if (ret != 0) {
			printf("sensor_channel_get error: %d\n", ret);
			break;
		}
		val_mv = (int32_t)buf;
		LOG_INF("RAW value: %d", val_mv);
		LOG_INF("CUSTOM value: %d",
			(val_mv * adc_ref_internal(adc_channel.dev)) >> adc_channel.resolution);

		ret = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
		if (ret < 0) {
			LOG_ERR("Raw to millivolts conversion error: %d", ret);
		} else {
			LOG_INF("ADC value: %d mV", val_mv);
			voltage_divider_scale_dt(&voltage_spec, &val_mv);
			LOG_INF("FULL value: %d mV", val_mv);
		}

		k_sleep(K_SECONDS(5));
	}
	return 0;
}
