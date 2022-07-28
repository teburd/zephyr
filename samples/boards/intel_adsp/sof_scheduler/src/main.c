/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/zephyr.h>
LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);

#define STACK_SIZE	2048
#define TIMER_PERIOD_US 100000

K_THREAD_STACK_ARRAY_DEFINE(work_stacks, CONFIG_MP_NUM_CPUS, STACK_SIZE);

struct zephyr_domain {
	struct k_thread domain_thread[CONFIG_MP_NUM_CPUS];
	struct k_sem sem[CONFIG_MP_NUM_CPUS];
};

struct k_timer timer;
static struct zephyr_domain zd = {};

void work_fn(void *arg1, void *arg2, void *arg3)
{
	int cpu = (int)arg1;
	struct zephyr_domain *zephyr_domain = arg2;

	while (true) {
		k_sem_take(&zephyr_domain->sem[cpu], K_FOREVER);

		/* Lets spinning to simulate some work */
		k_busy_wait(TIMER_PERIOD_US * 0.7);
	}
}

static uint32_t last_ccount = 0;

static void timer_fn(struct k_timer *t)
{
	int core;
	uint32_t enter_ccount, exit_ccount;
	__asm__ __volatile__("rsr %0,ccount":"=a" (enter_ccount));

	struct zephyr_domain *zephyr_domain = k_timer_user_data_get(t);

	if (!zephyr_domain) {
		return;
	}

	for (core = 0; core < CONFIG_MP_NUM_CPUS; core++) {
		k_sem_give(&zephyr_domain->sem[core]);
	}

	__asm__ __volatile__("rsr %0,ccount":"=a" (exit_ccount));

	/* Assume 400MHz cpu clock since none is defined in dts */
	uint32_t last_tdiff = (last_ccount - enter_ccount)/400;
	uint32_t tdiff = (exit_ccount - enter_ccount)/400;
	printk("CPU[%d] timer_fn took %u ns, time since last timer_fn %u ns\n", arch_curr_cpu()->id, tdiff, last_tdiff);	
	last_ccount = enter_ccount;
}

void main(void)
{
	
	LOG_INF("starting timer in 500ms!");
	k_msleep(500);	
	
	k_timeout_t start = {0};
	k_timeout_t timeout = K_USEC(TIMER_PERIOD_US);


	k_timer_init(&timer, timer_fn, NULL);
	k_timer_user_data_set(&timer, &zd);

	LOG_INF("number of ticks per second: %u\n", CONFIG_SYS_CLOCK_TICKS_PER_SEC);
	for (int i = 0; i < CONFIG_MP_NUM_CPUS; i++) {

		k_sem_init(&zd.sem[i], 0, 999);
		k_thread_create(&zd.domain_thread[i], work_stacks[i], STACK_SIZE, work_fn,
				(void *)i, &zd, NULL, -CONFIG_NUM_COOP_PRIORITIES, 0, K_FOREVER);
		k_thread_cpu_mask_clear(&zd.domain_thread[i]);
		k_thread_cpu_mask_enable(&zd.domain_thread[i], i);

		k_thread_start(&zd.domain_thread[i]);
	}
	k_timer_start(&timer, start, timeout);
}
