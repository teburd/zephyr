/*
 * Copyright (c) 2021 Intel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/battery.h>
#include <stdio.h>
#include <sys/util.h>

void main(void)
{
	int32_t voltage, current, charge, capacity, soc;
	const struct device *max17055_dev = DEVICE_DT_GET_ANY(max_max17055);

	if (max17055_dev == NULL) {
		printk("Could not get MAX17055 device\n");
		return;
	}

	while (1) {
		/* Erase previous */
		printk("\0033\014");
		printk("MAX17055:\n\n");
		printk("voltage: %d uV current %d uA\n", current, voltage);
		printk("charge: %d uAh capacity %d uAh\n", charge, capacity);
		printk("soc: %d \n", soc);
		k_sleep(K_MSEC(1000));
	}
}
