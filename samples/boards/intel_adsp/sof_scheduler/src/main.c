/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/zephyr.h>
LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);

#define STACK_SIZE	2048
#define TIMER_PERIOD_US 500000

K_THREAD_STACK_ARRAY_DEFINE(work_stacks, CONFIG_MP_NUM_CPUS, STACK_SIZE);

struct zephyr_domain {
	struct k_thread domain_thread[CONFIG_MP_NUM_CPUS];
	struct k_sem sem[CONFIG_MP_NUM_CPUS];
};

struct k_timer timer;
static struct zephyr_domain zd = {};

struct timer_isr_data {
	uint32_t cpu_id;
	uint32_t ccount;
	uint64_t tcount;
	uint64_t entries;
};

extern struct timer_isr_data timer_log[];
extern uint32_t timer_log_idx;

void work_fn(void *arg1, void *arg2, void *arg3)
{
	uint32_t last_ccount, enter_ccount, exit_ccount;
	int cpu = (int)arg1;
	struct zephyr_domain *zephyr_domain = arg2;

	__asm__ __volatile__("rsr %0,ccount":"=a" (last_ccount));

	while (true) {

		__asm__ __volatile__("rsr %0,ccount":"=a" (enter_ccount));

		k_sem_take(&zephyr_domain->sem[cpu], K_FOREVER);

		/* Lets spinning to simulate some work */
		k_busy_wait(TIMER_PERIOD_US * 0.7);

		__asm__ __volatile__("rsr %0,ccount":"=a" (exit_ccount));

		if (arch_curr_cpu()->id == 0)
		{
			printk("idx %d\n", timer_log_idx);

			if(timer_log_idx >= 1000) {
				for (int i = 0; i < 10; i++) {
					struct timer_isr_data *data = &timer_log[i];
					printk("cpu[%d] ccount(%u), tcount(%llu), entries (%llu)\n", data->cpu_id, data->ccount, data->tcount, data->entries);
				};
			}
		}
	}
}



static void timer_fn(struct k_timer *t)
{
	int core;

	struct zephyr_domain *zephyr_domain = k_timer_user_data_get(t);

	if (!zephyr_domain) {
		return;
	}

	for (core = 0; core < CONFIG_MP_NUM_CPUS; core++) {
		k_sem_give(&zephyr_domain->sem[core]);
	}
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
