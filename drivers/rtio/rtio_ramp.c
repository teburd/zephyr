/*
 * Copyright (c) 2019  Tom Burdick <tom.burdick@electromatic.us>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief RTIO Ramp Wave Generator
 * @defgroup rtio_interface RTIO Interface
 * @ingroup io_interfaces
 * @{
 */

#include <kernel.h>
#include <drivers/rtio.h>
#include <drivers/rtio/ramp.h>

#define LOG_DOMAIN "rtio_ramp"
#define LOG_LEVEL CONFIG_RTIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(rtio_ramp);

#ifdef __cplusplus
extern "C" {
#endif

struct rtio_ramp_data {
	struct rtio_driver_data rtio_data;
	struct rtio_ramp_config ramp_config;
	u32_t last_timestamp;
	u32_t cur_value;
	u32_t deltat_ns;
	u32_t remainder_ns;
};


static inline struct rtio_ramp_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

int rtio_ramp_configure(struct device *dev, struct rtio_config *config)
{
	struct rtio_ramp_data *dev_data = get_dev_data(dev);

	int res = rtio_begin_configuration(&dev_data->rtio_data);

	if (res != 0) {
		return res;
	}

	struct rtio_ramp_config *ramp_config = config->driver_config;

	memcpy(&dev_data->ramp_config, ramp_config,
	       sizeof(struct rtio_ramp_config));

	dev_data->deltat_ns = 1000000000/dev_data->ramp_config.sample_rate;
	LOG_DBG("Sample deltat_ns: %d", dev_data->deltat_ns);

	rtio_end_configuration(&dev_data->rtio_data, config);
	return res;
}

int rtio_ramp_trigger_read(struct device *dev, s32_t timeout)
{
	struct rtio_ramp_data *dev_data = get_dev_data(dev);
	struct rtio_block *block = NULL;

	int res = rtio_begin_trigger_read(&dev_data->rtio_data, &block, timeout);

	if (res != 0) {
		return res;
	}

	u32_t now = k_cycle_get_32();
	u32_t tstamp_diff = now - dev_data->last_timestamp;
	u32_t sampling_time = SYS_CLOCK_HW_CYCLES_TO_NS(tstamp_diff)
		+ dev_data->remainder_ns;

	LOG_DBG("Last cycle count %d, current cycle count %d, remainder time %d, sampling time %d, block address %d, block available %d", dev_data->last_timestamp, now, dev_data->remainder_ns, sampling_time, block, rtio_block_available(block));

	u32_t n = 0;
	while (sampling_time > dev_data->deltat_ns
			&& rtio_block_available(block) >= 4) {
		res = rtio_block_push_u32_t(block, dev_data->cur_value);
		if (res != 0) {
			break;
		}
		dev_data->cur_value += 1;
		if (dev_data->cur_value > dev_data->ramp_config.max_value) {
			dev_data->cur_value = 0;
		}
		sampling_time -= dev_data->deltat_ns;
		n += 1;
	}

	LOG_DBG("Generated %d samples, block available %d", n, rtio_block_available(block));

	dev_data->remainder_ns = sampling_time;
	dev_data->last_timestamp = now;

	rtio_end_trigger_read(&dev_data->rtio_data);

	return res;
}

static const struct rtio_api rtio_ramp_driver_api = {
	.configure = rtio_ramp_configure,
	.trigger_read = rtio_ramp_trigger_read,
};


int rtio_ramp_init(struct device *dev)
{
	/* statically initialized, so nothing to do yet but set the timestamp
	 * and add an appropriate counter callback to generate samples
	 */
	struct rtio_ramp_data *drv_data = dev->driver_data;

	drv_data->last_timestamp = k_cycle_get_32();
	drv_data->cur_value = 0;
	drv_data->ramp_config.sample_rate = 0;
	drv_data->ramp_config.max_value = 0;
	rtio_driver_data_init(&drv_data->rtio_data);
	return 0;
}

static struct rtio_ramp_data rtio_ramp_data;

DEVICE_AND_API_INIT(ramp, "RTIO_RAMP", rtio_ramp_init,
		    &rtio_ramp_data, NULL, POST_KERNEL,
		    CONFIG_RTIO_INIT_PRIORITY, &rtio_ramp_driver_api);


#ifdef __cplusplus
}
#endif
