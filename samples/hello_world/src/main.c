/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>

static uint32_t busy = 0;
void timer_fn(struct k_timer *tm)
{
	k_sem_give()
}

K_TIMER_DEFINE(timer, timer_fn, NULL);

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	k_sleep(K_MSEC(100));
	k_timer_start(&timer, K_MSEC(100), K_MSEC(100));
}
