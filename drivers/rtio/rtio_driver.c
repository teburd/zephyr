/*
 * Copyright (c) 2019  Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief RTIO Common Driver Functions
 * @defgroup rtio_interface RTIO Interface
 * @ingroup io_interfaces
 * @{
 */

#include <kernel.h>
#include <drivers/rtio.h>

#ifdef __cplusplus
extern "C" {
#endif


void rtio_output_config_init(struct rtio_output_config *cfg)
{
	cfg->allocator = NULL;
	cfg->fifo = NULL;
	cfg->timeout = K_FOREVER;
	cfg->byte_size = 0;
}


void rtio_config_init(struct rtio_config *cfg)
{
	rtio_output_config_init(&cfg->output_config);
	cfg->driver_config = NULL;
}

void rtio_driver_data_init(struct rtio_driver_data *data)
{
	k_sem_init(&data->sem, 0, 1);
	rtio_config_init(&data->config);
	k_timer_init(&data->output_timer, rtio_output_timeout, NULL);
	data->block = NULL;
	k_sem_give(&data->sem);
}

int rtio_begin_configuration(struct rtio_driver_data *data)
{
	k_sem_take(&data->sem, K_FOREVER);
	if (data->block != NULL && data->config.output_config.fifo != NULL) {
		k_fifo_put(data->config.output_config.fifo, data->block);
		data->block = NULL;
		return 1;
	}
	return 0;
}

void rtio_end_configuration(struct rtio_driver_data *data,
			    struct rtio_config *config)
{
	memcpy(&data->config, config, sizeof(struct rtio_config));

	/* Setup timer if needed */
	k_timer_stop(&data->output_timer);
	if (data->config.output_config.timeout != K_FOREVER
	   || data->config.output_config.timeout != K_NO_WAIT) {
		u32_t timeout_ns = SYS_CLOCK_HW_CYCLES_TO_NS(data->config.output_config.timeout);
		s32_t timeout_ms = timeout_ns/1000000;

		k_timer_start(&data->output_timer, timeout_ms, timeout_ms);
	}

	k_sem_give(&data->sem);
}

int rtio_begin_trigger_read(struct rtio_driver_data *data,
			    struct rtio_block **block,
			    s32_t timeout)
{
	int res = 0;

	res = k_sem_take(&data->sem, timeout);
	if (res != 0) {
		return -EBUSY;
	}

	if (data->block == NULL) {
		res = rtio_block_alloc(data->config.output_config.allocator,
				       &data->block,
				       data->config.output_config.byte_size,
				       timeout);
		if (res != 0) {
			goto error;
		}
	}

	*block = data->block;
	return res;

error:
	k_sem_give(&data->sem);
	return res;

}

int rtio_end_trigger_read(struct rtio_driver_data *data)
{
	int res = rtio_output_block(data);

	k_sem_give(&data->sem);
	return res;
}

void rtio_output_timeout(struct k_timer *timer)
{
	struct rtio_driver_data *data =
		CONTAINER_OF(timer, struct rtio_driver_data, output_timer);

	int res = k_sem_take(&data->sem, K_NO_WAIT);

	if (res == 0) {
		if (data->block != NULL && data->config.output_config.fifo != NULL) {
			k_fifo_put(data->config.output_config.fifo, data->block);
			data->block = NULL;
		}
		k_sem_give(&data->sem);
	}
}


#ifdef __cplusplus
}
#endif
