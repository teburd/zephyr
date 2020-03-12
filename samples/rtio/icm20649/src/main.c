/*
 * Copyright (c) 2020 Tom Burdick <tom.burdick@electromatic.us>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/rtio.h>
#include <drivers/rtio/sensor/icm20649.h>
#include <stdio.h>
#include <sys/printk.h>
#include <sys/__assert.h>

#define LOG_DOMAIN "icm20649_reader"
#define LOG_LEVEL CONFIG_RTIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ramp_reader);


#define MAX_BLOCK_SIZE 4096
#define MAX_BLOCKS 4

RTIO_MEMPOOL_ALLOCATOR_DEFINE(blockalloc, 64, MAX_BLOCK_SIZE, MAX_BLOCKS, 4);
K_FIFO_DEFINE(icm20649_out_fifo);

volatile static u32_t triggers, ebusy, enomem, eagain;

void trigger_read(void *s1, void *s2, void *s3)
{
	struct device *ramp_dev = (struct device *)s1;
	int res = 0;

	while (true) {
		res = rtio_trigger_read(ramp_dev, K_NO_WAIT);
		triggers += 1;
		switch (res) {
		case -EBUSY:
			ebusy += 1;
			break;
		case -ENOMEM:
			enomem += 1;
			break;
		case -EAGAIN:
			eagain += 1;
			break;
		default:
			break;
		}
		if (res != 0) {
			k_yield();
		}
	}
}

#define TRIGGER_READ_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(trigger_read_stack, TRIGGER_READ_STACK_SIZE);
struct k_thread trigger_read_thread;

int main(void)
{
	printk("RTIO ICM20649 Sample\n");

	struct device *icm20649_dev  = device_get_binding("RTIO_SENSOR_ICM20649");
	__ASSERT_NO_MSG(icm20649_dev != NULL);

	/** Maximum range and rate with fifo configuration for icm20649 */
	struct icm20649_config icm20649_cfg = {
		.reset = true,
		.accel_config = {
			.enabled = true,
			.self_test = false,
			.low_power_mode = false,
			.filter = ICM20649_ACCEL_LPF_473_0_HZ,
			.scale = ICM20649_30_G,
			.sample_rate_div = 0,

		},
		.gyro_config = {
			.enabled = true,
			.self_test = false,
			.low_power_mode = false,
			.filter = ICM20649_GYRO_LPF_376_5_HZ,
			.scale = ICM20649_4000_DPS,
			.sample_rate_div = 0,

		},
		.temp_config = {
			.enabled = true,
			.filter = ICM20649_TEMP_LPF_217_9_HZ,
		},
		.fifo_enabled = true,
	};

	struct rtio_config config =  {
		.output_config = {
			.allocator = blockalloc,
			.fifo = &ramp_out_fifo,
			.timeout = K_FOREVER,
			.byte_size = MAX_BLOCK_SIZE
		},
		.driver_config = &icm20649_cfg
	};

	triggers = 0;
	ebusy = 0;
	enomem = 0;
	eagain = 0;

	int res = rtio_configure(icm20649_dev, &config);

	__ASSERT_NO_MSG(res == 0);

	k_tid_t trigger_tid = k_thread_create(&trigger_read_thread,
					      trigger_read_stack,
					      K_THREAD_STACK_SIZEOF(trigger_read_stack),
					      trigger_read,
					      ramp_dev, NULL, NULL,
					      0, 0, K_NO_WAIT);

	u32_t blocks = 0;
	u32_t samples = 0;
	u32_t bytes = 0;
	u32_t tstamp = k_cycle_get_32();
	u64_t last_print = tstamp;

	while (true) {
		struct rtio_block *block = k_fifo_get(&icm20649_out_fifo, K_FOREVER);
		blocks += 1;
		bytes += rtio_block_used(block);
		samples += rtio_block_used(block)/sizeof(u32_t);

		u32_t now = k_cycle_get_32();
		u64_t last_print_diff = k_cyc_to_ns_floor64(now - last_print);
		if (last_print_diff > 1000000000) {
			float tdiff = last_print_diff/1000000000.0;
			float block_rate = blocks/tdiff;
			float byte_rate = bytes/tdiff;
			float sample_rate = samples/tdiff;
			printf("time delta %f, %d blocks, %f blocks/sec, %d bytes, %f bytes/sec, %d samples, %f samples/sec\n", tdiff, blocks, block_rate, bytes, byte_rate, samples, sample_rate);
			printf("read triggers %d, ebusy %d, enomem %d, eagain %d\n", triggers, ebusy, enomem, eagain);
			last_print = now;
			blocks = 0;
			samples = 0;
			bytes = 0;
		}

		/*
		for (size_t i = 0; i < rtio_block_used(block); i += sizeof(u32_t)) {
			printk("%d", *((u32_t *)(block->data + i)));
			if ((i % 128 == 0 && i > 0) || i + sizeof(u32_t) >= rtio_block_used(block)) {
				printk("\n\t");
			} else {
				printk(", ");
			}
		}
		*/
		rtio_block_free(blockalloc, block);
		tstamp = now;
	}
}
